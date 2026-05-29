#!/usr/bin/env python3
"""
ZMQ Client for Groot2 protocol.
Connects to BehaviorTree.CPP Groot2Publisher and parses tree structure + status updates.
"""

import zmq
import json
import time
import threading
import struct
import os
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Callable


class Groot2Client:
    """Client for Groot2Publisher ZMQ protocol."""
    
    def __init__(self, 
                 zmq_host: str = "localhost",
                 zmq_publisher_port: int = 1666,
                 zmq_server_port: int = 1667,
                 logger=None):
        """
        Initialize Groot2 client.
        
        Args:
            zmq_host: Hostname/IP of brain_node
            zmq_publisher_port: Port for status updates stream (typically 1666)
            zmq_server_port: Port for tree structure requests (typically 1667)
            logger: ROS logger instance
        """
        self.zmq_host = zmq_host
        self.zmq_publisher_port = zmq_publisher_port
        self.zmq_server_port = zmq_server_port
        self.logger = logger
        
        self.context = zmq.Context()
        self.requester = None
        self.connected = False
        
        # Tree data
        self.tree_structure = {"nodes": [], "edges": []}
        self.node_status = {}  # node_id -> status
        
        # Blackboard data
        self.blackboard_data = {}  # key -> value
        self._bb_supported = True  # Set False if Groot2 doesn't support 'B' request
        
        # Callbacks
        self.on_tree_update: Optional[Callable] = None
        self.on_status_update: Optional[Callable] = None
        
        # Thread control
        self._running = False
        self._thread = None
        
    def connect(self) -> bool:
        """Establish ZMQ connections."""
        try:
            # Requester for tree structure and status (REQ-REP pattern)
            self.requester = self.context.socket(zmq.REQ)
            self.requester.connect(f"tcp://{self.zmq_host}:{self.zmq_server_port}")
            self.requester.setsockopt(zmq.RCVTIMEO, 2000)  # 2 second timeout
            
            self.connected = True
            if self.logger:
                self.logger.info(f"Connected to Groot2Publisher at {self.zmq_host}:{self.zmq_server_port}")
            return True
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to connect to ZMQ: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Close ZMQ connections."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        
        if self.requester:
            self.requester.close()
        
        self.connected = False
        if self.logger:
            self.logger.info("Disconnected from Groot2Publisher")
    
    def _reconnect_requester(self):
        """Recreate the REQ socket after a failed request leaves it in bad state.
        
        ZMQ REQ sockets alternate between send/recv states. If a recv times out
        after a send, the socket is stuck in 'recv' state and can't send again.
        The only fix is to close and recreate the socket.
        """
        try:
            if self.requester:
                self.requester.close()
            self.requester = self.context.socket(zmq.REQ)
            self.requester.connect(f"tcp://{self.zmq_host}:{self.zmq_server_port}")
            self.requester.setsockopt(zmq.RCVTIMEO, 2000)
            if self.logger:
                self.logger.debug("Reconnected REQ socket after blackboard failure")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to reconnect REQ socket: {e}")
            self.connected = False
    
    def request_tree_structure(self) -> bool:
        """
        [DEPRECATED] Request full tree structure from server via Groot2 protocol.
        
        DO NOT CALL THIS METHOD DIRECTLY!
        Tree structure is now requested automatically inside the monitor thread.
        This avoids ZMQ REQ socket state conflicts.
        """
        if not self.connected:
            return False
        
        try:
            # Build Groot2 request header (6 bytes) with little-endian
            protocol = 2  # kProtocolID
            req_type = ord('T')  # FULLTREE
            unique_id = struct.unpack('I', os.urandom(4))[0] & 0xFFFFFFFF
            header = struct.pack('<BBI', protocol, req_type, unique_id)
            
            # Send request
            self.requester.send(header)
            
            # Receive multipart response
            response = self.requester.recv_multipart()
            
            if len(response) < 2:
                if self.logger:
                    self.logger.error("Invalid response format from Groot2Publisher")
                return False
            
            # response[0] = header (22 bytes)
            # response[1] = XML string of complete tree
            xml_string = response[1].decode('utf-8')
            
            # Parse XML tree
            tree_data = self._parse_tree_xml(xml_string)
            
            if tree_data:
                self.tree_structure = tree_data
                if self.on_tree_update:
                    self.on_tree_update(tree_data)
                if self.logger:
                    self.logger.info(f"Tree structure received: {len(tree_data.get('nodes', []))} nodes")
                return True
                
        except zmq.Again:
            if self.logger:
                self.logger.warn("Timeout requesting tree structure")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Error requesting tree: {e}")
        
        return False
    
    def _parse_tree_xml(self, xml_string: str) -> Optional[Dict]:
        """
        Parse tree structure from Groot2 XML response.
        
        Groot2 sends XML (not flatbuffers!) containing all BehaviorTrees and their nodes.
        """
        try:
            # Check if string is empty
            if not xml_string or xml_string.strip() == '':
                if self.logger:
                    self.logger.error("Received empty XML string")
                return None
            
            root = ET.fromstring(xml_string)
            
            nodes = []
            edges = []
            node_uid_map = {}  # uid -> node_data
            
            # Parse all BehaviorTree elements
            for bt_elem in root.findall('.//BehaviorTree'):
                bt_id = bt_elem.get('ID', 'UnknownTree')
                
                # Process all nodes in this tree
                for elem in bt_elem.iter():
                    if elem == bt_elem:
                        continue  # Skip the BehaviorTree element itself
                    
                    uid = elem.get('_uid')
                    if uid:
                        uid = int(uid)
                        
                        # Get both ID and name attributes
                        node_id = elem.get('ID', '')
                        node_name = elem.get('name', '')
                        
                        # Name resolution priority:
                        # 1. ID — always the C++ class name, always English
                        # 2. name with Chinese stripped — keep only ASCII parts
                        # 3. XML tag name (elem.tag)
                        display_name = node_id if node_id else self._sanitize_name(node_name, elem.tag)
                        
                        node_type = elem.tag
                        
                        # Special handling for SubTree nodes
                        subtree_id = elem.get('ID') if node_type == 'SubTree' else None
                        
                        # Extract ports/parameters (all attributes except internal ones)
                        internal_attrs = {'_uid', 'ID', 'name', '_fullpath'}
                        ports = {}
                        for attr_name, attr_value in elem.attrib.items():
                            if attr_name not in internal_attrs:
                                ports[attr_name] = attr_value
                        
                        node_data = {
                            'uid': uid,
                            'name': display_name,
                            'type': node_type,
                            'tree': bt_id,
                            'children': [],
                            'subtree_ref': subtree_id,  # Reference to subtree ID
                            'ports': ports  # BT ports/parameters
                        }
                        
                        nodes.append(node_data)
                        node_uid_map[uid] = node_data
            
            # Build parent-child relationships
            for bt_elem in root.findall('.//BehaviorTree'):
                for parent_elem in bt_elem.iter():
                    parent_uid = parent_elem.get('_uid')
                    if parent_uid:
                        parent_uid = int(parent_uid)
                        parent_node = node_uid_map.get(parent_uid)
                        
                        if parent_node:
                            for child_elem in parent_elem:
                                child_uid = child_elem.get('_uid')
                                if child_uid:
                                    child_uid = int(child_uid)
                                    parent_node['children'].append(child_uid)
                                    
                                    # Create edge
                                    edges.append({
                                        'from': parent_uid,
                                        'to': child_uid
                                    })
            
            if self.logger:
                self.logger.info(f"Parsed {len(nodes)} nodes from XML")
            
            return {
                'nodes': nodes,
                'edges': edges,
                'nodes_by_uid': node_uid_map
            }
            
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to parse tree XML: {e}")
            return None
    
    def _sanitize_name(self, name: str, fallback: str = 'Node') -> str:
        """Sanitize node name by stripping non-ASCII (Chinese) characters.
        
        Keeps only English letters, digits, and common symbols.
        Falls back to the XML tag name if nothing useful remains.
        """
        import re
        
        if not name:
            return fallback
        
        # Keep only ASCII printable characters (letters, digits, punctuation)
        ascii_only = re.sub(r'[^\x20-\x7E]', '', name)
        
        # Clean up artifacts: multiple spaces, leading/trailing junk
        ascii_only = re.sub(r'\s+', ' ', ascii_only).strip()
        
        # Remove orphaned brackets/arrows: "[] ->" etc.
        ascii_only = re.sub(r'^\[?\]?\s*->\s*', '', ascii_only)
        ascii_only = re.sub(r'\s*->\s*\[?\]?\s*$', '', ascii_only)
        
        # If nothing useful remains, use fallback
        if not ascii_only or len(ascii_only) < 2:
            return fallback
        
        return ascii_only
    
    def start_monitoring(self):
        """Start background thread to monitor status updates."""
        if self._running:
            return
        
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()
        
        if self.logger:
            self.logger.info("Started status monitoring")
    
    def _monitor_loop(self):
        """
        Background loop:
        1. Request tree structure once at start
        2. Poll status updates continuously
        
        IMPORTANT: All ZMQ REQ-REP must happen in this thread to avoid state conflicts!
        """
        # First, request tree structure (only once)
        try:
            protocol = 2
            req_type = ord('T')  # FULLTREE
            unique_id = struct.unpack('I', os.urandom(4))[0] & 0xFFFFFFFF
            
            # Build header with explicit little-endian format
            header = struct.pack('<BBI', protocol, req_type, unique_id)
            
            if self.logger:
                self.logger.info(f"Requesting tree with header: protocol={protocol}, type={chr(req_type)}, id={unique_id}")
            
            self.requester.send(header)
            response = self.requester.recv_multipart()
            
            if len(response) >= 2:
                xml_string = response[1].decode('utf-8')
                
                # Debug: log first 200 chars of XML
                if self.logger:
                    preview = xml_string[:200] if len(xml_string) > 200 else xml_string
                    self.logger.info(f"XML preview: {preview}")
                
                tree_data = self._parse_tree_xml(xml_string)
                
                if tree_data:
                    self.tree_structure = tree_data
                    if self.on_tree_update:
                        self.on_tree_update(tree_data)
                    if self.logger:
                        self.logger.info(f"Tree structure received: {len(tree_data.get('nodes', []))} nodes")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to get tree structure: {e}")
            return  # Exit thread if we can't get tree
        
        # Main monitoring loop
        poll_count = 0
        while self._running:
            if not self.connected:
                time.sleep(0.1)
                continue
            
            try:
                # Request status from Groot2Publisher
                statuses = self._request_status()
                
                if statuses:
                    # Send ALL statuses, not just changes
                    # This ensures RUNNING nodes are always visible
                    for uid, status in statuses.items():
                        old_status = self.node_status.get(uid)
                        
                        # Always update and notify for RUNNING, SUCCESS, FAILURE,
                        # and IDLE_FROM_* (carry transition info for faded colors).
                        # Only skip plain IDLE if it hasn't changed.
                        should_notify = (
                            status in ['RUNNING', 'SUCCESS', 'FAILURE',
                                       'IDLE_FROM_SUCCESS', 'IDLE_FROM_FAILURE',
                                       'IDLE_FROM_RUNNING'] or
                            old_status != status
                        )
                        
                        if should_notify:
                            self.node_status[uid] = status
                            
                            # Notify callback
                            if self.on_status_update:
                                self.on_status_update({
                                    'node_id': uid,
                                    'status': status
                                })
                
                # Request blackboard data every ~1 second (every 10 polls)
                poll_count += 1
                if poll_count % 10 == 0:
                    self._request_blackboard()
                
                # Poll at 10 Hz (fast enough to catch everything)
                time.sleep(0.1)
                
            except zmq.Again:
                time.sleep(0.1)
            except Exception as e:
                if self.logger and self._running:
                    self.logger.error(f"Error in monitor loop: {e}")
                time.sleep(0.5)
    
    def _request_status(self) -> Optional[Dict[int, str]]:
        """Request current status of all nodes via Groot2 protocol."""
        try:
            # Build Groot2 request header (6 bytes) with little-endian
            protocol = 2  # kProtocolID
            req_type = ord('S')  # STATUS
            unique_id = struct.unpack('I', os.urandom(4))[0] & 0xFFFFFFFF
            header = struct.pack('<BBI', protocol, req_type, unique_id)
            
            # Send request
            self.requester.send(header)
            
            # Receive multipart response
            response = self.requester.recv_multipart()
            
            if len(response) < 2:
                return None
            
            # response[0] = header (22 bytes)
            # response[1] = binary status buffer
            status_buffer = response[1]
            
            # Parse binary status buffer
            # Format: [uint16 uid][uint8 status] repeated for each node
            statuses = {}
            offset = 0
            
            while offset + 3 <= len(status_buffer):
                uid = struct.unpack_from('H', status_buffer, offset)[0]
                status_byte = status_buffer[offset + 2]
                statuses[uid] = self._decode_status(status_byte)
                offset += 3
            
            return statuses
            
        except zmq.Again:
            # Timeout is normal during polling
            return None
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to request status: {e}")
            return None
    
    def _request_blackboard(self):
        """Request blackboard data from Groot2Publisher.
        
        Tries the Groot2 'B' (Blackboard) request type.
        If the BT library version doesn't support it, falls back to
        deriving blackboard data from the node status summary.
        
        IMPORTANT: If the 'B' request fails, we disable future attempts
        because the ZMQ REQ socket gets stuck in S:SENT state on timeout,
        which would break subsequent status requests.
        """
        if not self._bb_supported:
            self._derive_blackboard_from_status()
            return
        
        try:
            # Build Groot2 request header (6 bytes) with little-endian
            protocol = 2  # kProtocolID
            req_type = ord('B')  # BLACKBOARD request
            unique_id = struct.unpack('I', os.urandom(4))[0] & 0xFFFFFFFF
            header = struct.pack('<BBI', protocol, req_type, unique_id)
            
            # Use shorter timeout for blackboard to avoid blocking status polls
            self.requester.setsockopt(zmq.RCVTIMEO, 500)  # 500ms timeout
            
            # Send request
            self.requester.send(header)
            
            # Receive response
            response = self.requester.recv_multipart()
            
            # Restore normal timeout
            self.requester.setsockopt(zmq.RCVTIMEO, 2000)
            
            if len(response) >= 2:
                # Parse blackboard XML or JSON response
                bb_raw = response[1].decode('utf-8', errors='replace')
                
                # Groot2Publisher returns XML with blackboard entries
                try:
                    bb_root = ET.fromstring(bb_raw)
                    new_bb = {}
                    for entry in bb_root.iter():
                        if entry.tag == 'Entry' or entry.tag == 'entry':
                            key = entry.get('key', entry.get('name', ''))
                            value = entry.get('value', entry.text or '')
                            if key:
                                new_bb[key] = value
                        elif entry.tag not in ('Blackboard', 'blackboard', ''):
                            # Some formats use tag as key, text as value
                            if entry.text:
                                new_bb[entry.tag] = entry.text
                    
                    if new_bb:
                        self.blackboard_data = new_bb
                except ET.ParseError:
                    # Not XML — try as plain text key=value pairs
                    new_bb = {}
                    for line in bb_raw.strip().split('\n'):
                        if '=' in line:
                            k, _, v = line.partition('=')
                            new_bb[k.strip()] = v.strip()
                    if new_bb:
                        self.blackboard_data = new_bb
                        
        except zmq.Again:
            # Timeout — blackboard request not supported.
            # CRITICAL: Disable future attempts to avoid socket corruption.
            self._bb_supported = False
            self.requester.setsockopt(zmq.RCVTIMEO, 2000)  # Restore timeout
            if self.logger:
                self.logger.info("Blackboard request not supported by Groot2Publisher. Using derived data.")
            # Need to recreate socket since REQ is now in bad state
            self._reconnect_requester()
            self._derive_blackboard_from_status()
        except Exception:
            # Silently fail — blackboard is optional
            self._bb_supported = False
            self.requester.setsockopt(zmq.RCVTIMEO, 2000)  # Restore timeout
            self._reconnect_requester()
            self._derive_blackboard_from_status()
    
    def _derive_blackboard_from_status(self):
        """Derive basic blackboard-like data from current node statuses.
        
        Fallback when the Groot2Publisher doesn't support blackboard requests.
        Provides useful execution summary information.
        """
        running_count = sum(1 for s in self.node_status.values() if s == 'RUNNING')
        success_count = sum(1 for s in self.node_status.values() if s == 'SUCCESS')
        failure_count = sum(1 for s in self.node_status.values() if s == 'FAILURE')
        idle_count = sum(1 for s in self.node_status.values() 
                        if s in ('IDLE', 'IDLE_FROM_SUCCESS', 'IDLE_FROM_FAILURE', 'IDLE_FROM_RUNNING'))
        total = len(self.node_status)
        
        self.blackboard_data = {
            '_running_nodes': str(running_count),
            '_success_nodes': str(success_count),
            '_failure_nodes': str(failure_count),
            '_idle_nodes': str(idle_count),
            '_total_nodes': str(total),
        }
    
    def _decode_status(self, status_byte: int) -> str:
        """Decode status byte to string.
        
        Groot2 status bytes 10/11/12 indicate IDLE transitions.
        We expose these as IDLE_FROM_* so the frontend can show
        correct faded colors (Groot1 visual memory technique).
        """
        status_map = {
            0: 'IDLE',
            1: 'RUNNING',
            2: 'SUCCESS',
            3: 'FAILURE',
            4: 'SKIPPED',
            10: 'IDLE_FROM_SUCCESS',
            11: 'IDLE_FROM_FAILURE',
            12: 'IDLE_FROM_RUNNING',
        }
        return status_map.get(status_byte, 'UNKNOWN')
    
    def get_tree_structure(self) -> Dict:
        """Get current tree structure."""
        return self.tree_structure
    
    def get_node_status(self, node_id: int) -> Optional[str]:
        """Get status of specific node."""
        return self.node_status.get(node_id)
    
    def get_all_statuses(self) -> Dict[int, str]:
        """Get all node statuses."""
        return self.node_status.copy()
    
    def get_blackboard_data(self) -> Dict:
        """Get current blackboard data.
        
        Returns a dict of key-value pairs from the BT blackboard.
        Values are converted to strings for JSON serialization.
        """
        return self.blackboard_data.copy()
