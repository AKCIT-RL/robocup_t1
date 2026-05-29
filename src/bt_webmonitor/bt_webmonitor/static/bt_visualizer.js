/**
 * BehaviorTree Visualizer — Groot1-inspired
 * 
 * Features:
 * - SVG gradients for RUNNING/SUCCESS/FAILURE (Groot1 style)
 * - Sticky status: SUCCESS/FAILURE stay vibrant for 1.5s, then faded for 1.5s
 * - IDLE_FROM_* status codes for accurate faded colors
 * - Active path highlighting (root → RUNNING nodes)
 * - Bezier cubic curves for connections
 * - Different shapes per node type (diamond=control, hexagon=decorator, rect=action)
 * - Node detail panel on click
 * - Collapse/expand subtrees (double-click)
 * - Blackboard viewer
 * - Keyboard shortcuts
 * - Throttled action feed (no RUNNING spam)
 */
class BtVisualizer {
    constructor() {
        this.ws = null;
        this.treeData = null;
        this.nodeStatuses = {};
        this.svg = null;
        this.g = null;
        this.zoom = null;
        this.startTime = Date.now();
        this.updateCounter = 0;
        this.lastRateCalc = Date.now();
        this.currentRate = 0;
        
        // Groot1 visual persistence
        this.visualStates = {};  // {uid: {class: 'success', timer1: id, timer2: id}}
        
        // Hierarchy & rendering state
        this.hierarchyRoot = null;
        this.d3Root = null;
        this.nodeElements = null;
        this.linkElements = null;
        this.collapsedSubtrees = new Set();
        
        // Selected node for detail panel
        this.selectedNodeUid = null;
        
        // Last action feed entries (for dedup)
        this.lastFeedEntries = {};  // {node_id: status}
        
        // Tooltip element
        this.tooltip = null;

        // Blackboard polling interval
        this._blackboardInterval = null;
        
        this.init();
    }
    
    init() {
        this.svg = d3.select("#tree-svg");
        this.tooltip = document.getElementById('bt-tooltip');
        
        // Add SVG gradients (Groot1-style 4-stop vertical gradients)
        const defs = this.svg.append("defs");
        this._createGradient(defs, "gradient-running", [
            {offset: "0%", color: "#FFD54F"},
            {offset: "3%", color: "#FFC107"},
            {offset: "97%", color: "#FF8F00"},
            {offset: "100%", color: "#F57C00"}
        ]);
        this._createGradient(defs, "gradient-success", [
            {offset: "0%", color: "#66BB6A"},
            {offset: "3%", color: "#4CAF50"},
            {offset: "97%", color: "#388E3C"},
            {offset: "100%", color: "#2E7D32"}
        ]);
        this._createGradient(defs, "gradient-failure", [
            {offset: "0%", color: "#EF5350"},
            {offset: "3%", color: "#F44336"},
            {offset: "97%", color: "#D32F2F"},
            {offset: "100%", color: "#C62828"}
        ]);
        
        // Zoom behavior
        this.zoom = d3.zoom()
            .scaleExtent([0.05, 4])
            .on("zoom", (event) => {
                this.g.attr("transform", event.transform);
            });
        
        this.svg.call(this.zoom);
        this.g = this.svg.append("g");
        
        // Controls
        d3.select("#btn-zoom-in").on("click", () => this.zoomIn());
        d3.select("#btn-zoom-out").on("click", () => this.zoomOut());
        d3.select("#btn-fit").on("click", () => this.fitToScreen());
        d3.select("#btn-reset").on("click", () => this.resetView());
        d3.select("#search-box").on("input", (event) => this.searchNodes(event.target.value));
        d3.select("#btn-collapse-all").on("click", () => this.collapseAll());
        d3.select("#btn-expand-all").on("click", () => this.expandAll());
        
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => this._handleKeyboard(e));
        
        // Tooltip hide on click outside
        this.svg.on("click", (event) => {
            if (event.target === this.svg.node()) {
                this._hideTooltip();
            }
        });
        
        // Connect WebSocket
        this.connectWebSocket();
        
        // Update stats & blackboard periodically
        setInterval(() => this.updateStats(), 1000);
        this._blackboardInterval = setInterval(() => this.fetchBlackboard(), 2000);
    }
    
    _createGradient(defs, id, stops) {
        const gradient = defs.append("linearGradient")
            .attr("id", id)
            .attr("x1", "0%").attr("y1", "0%")
            .attr("x2", "0%").attr("y2", "100%");
        stops.forEach(s => {
            gradient.append("stop")
                .attr("offset", s.offset)
                .attr("stop-color", s.color);
        });
    }
    
    _handleKeyboard(e) {
        // Don't trigger shortcuts when typing in search
        if (e.target.tagName === 'INPUT') return;
        
        switch (e.key) {
            case 'f': case 'F': this.fitToScreen(); break;
            case 'r': case 'R': this.resetView(); break;
            case '+': case '=': this.zoomIn(); break;
            case '-': case '_': this.zoomOut(); break;
        }
    }
    
    // =================================================================
    // WebSocket
    // =================================================================
    
    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws/bt_monitor`;
        
        this.setConnectionStatus('connecting', 'Connecting...');
        
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            this.setConnectionStatus('connected', 'Connected');
        };
        
        this.ws.onclose = () => {
            this.setConnectionStatus('disconnected', 'Disconnected');
            setTimeout(() => this.connectWebSocket(), 3000);
        };
        
        this.ws.onerror = () => {
            // Error will be followed by onclose
        };
        
        this.ws.onmessage = (event) => {
            try {
                const message = JSON.parse(event.data);
                this.handleMessage(message);
            } catch (err) {
                // Ignore malformed messages
            }
        };
    }
    
    setConnectionStatus(status, text) {
        const indicator = document.getElementById('status-indicator');
        const statusText = document.getElementById('status-text');
        
        indicator.className = `status-dot ${status}`;
        statusText.textContent = text;
        
        document.getElementById('ws-status').textContent = text;
    }
    
    handleMessage(message) {
        switch (message.type) {
            case 'tree_structure':
                this.updateTree(message.data);
                break;
            case 'status_update':
                // Legacy single update
                this.updateNodeStatus(message.data);
                break;
            case 'status_batch':
                // Batch update (efficient)
                this._processBatch(message.data);
                break;
            case 'heartbeat':
                break;
        }
    }
    
    // =================================================================
    // Tree Structure
    // =================================================================
    
    updateTree(treeData) {
        this.treeData = treeData;
        document.getElementById('no-data').style.display = 'none';
        
        if (treeData.nodes && treeData.nodes.length > 0) {
            this.renderTree();
        }
    }
    
    buildHierarchy(nodes, nodesByUid) {
        if (!nodes || nodes.length === 0) return null;
        
        // Find the root node
        const childUids = new Set();
        nodes.forEach(n => (n.children || []).forEach(c => childUids.add(c)));
        let root = nodes.find(n => !childUids.has(n.uid)) || nodes[0];
        
        const visited = new Set();
        
        const buildNode = (uid) => {
            if (visited.has(uid)) return null;
            visited.add(uid);
            
            const node = nodesByUid[uid];
            if (!node) return null;
            
            const isCollapsed = this.collapsedSubtrees.has(uid);
            
            // SubTree expansion
            if (node.type === 'SubTree' && node.subtree_ref && !isCollapsed) {
                const subtreeNodes = nodes.filter(n => n.tree === node.subtree_ref);
                const subtreeChildUids = new Set();
                subtreeNodes.forEach(n => (n.children || []).forEach(c => subtreeChildUids.add(c)));
                const subtreeRoot = subtreeNodes.find(n => !subtreeChildUids.has(n.uid));
                
                if (subtreeRoot) {
                    const subtreeChildren = (subtreeRoot.children || [])
                        .map(childUid => buildNode(childUid))
                        .filter(Boolean);
                    
                    return {
                        ...node,
                        displayName: node.name,
                        children: subtreeChildren,
                        isExpandedSubtree: true,
                        _collapsed: false
                    };
                }
            }
            
            // Normal node
            const children = isCollapsed ? [] :
                (node.children || []).map(childUid => buildNode(childUid)).filter(Boolean);
            
            return {
                ...node,
                displayName: node.name || node.type,
                children: children,
                _collapsed: isCollapsed && (node.children || []).length > 0
            };
        };
        
        return buildNode(root.uid);
    }
    
    renderTree() {
        if (!this.treeData || !this.treeData.nodes || this.treeData.nodes.length === 0) {
            return;
        }
        
        this.g.selectAll("*").remove();
        
        // Build node lookup
        const nodesByUid = {};
        this.treeData.nodes.forEach(n => { nodesByUid[n.uid] = n; });
        
        const hierarchyRoot = this.buildHierarchy(this.treeData.nodes, nodesByUid);
        if (!hierarchyRoot) return;
        
        this.hierarchyRoot = hierarchyRoot;
        this.d3Root = d3.hierarchy(hierarchyRoot);
        
        // Groot1 layout constants
        const NODE_SPACING = 80;
        const LEVEL_SPACING = 120;
        
        const nodeCount = this.d3Root.descendants().length;
        const treeDepth = this.d3Root.height;
        
        const treeWidth = Math.max(1200, nodeCount * NODE_SPACING);
        const treeHeight = Math.max(600, treeDepth * LEVEL_SPACING + 200);
        
        const treeLayout = d3.tree()
            .size([treeWidth, treeHeight])
            .separation((a, b) => {
                if (a.parent === b.parent) return 1.2;
                return 1.8;
            });
        
        treeLayout(this.d3Root);
        
        // Draw links (Bezier cubic curves — Groot1 style)
        this.linkElements = this.g.selectAll(".tree-link")
            .data(this.d3Root.links())
            .join("path")
            .attr("class", "tree-link")
            .attr("d", d => {
                const sx = d.source.x, sy = d.source.y;
                const tx = d.target.x, ty = d.target.y;
                const midY = (sy + ty) / 2;
                return `M${sx},${sy} C${sx},${midY} ${tx},${midY} ${tx},${ty}`;
            });
        
        // Draw nodes
        const self = this;
        this.nodeElements = this.g.selectAll(".tree-node")
            .data(this.d3Root.descendants())
            .join("g")
            .attr("class", d => {
                const vs = this.visualStates[d.data.uid];
                const cls = vs ? vs.class : 'idle';
                return `tree-node ${cls}`;
            })
            .attr("transform", d => `translate(${d.x},${d.y})`)
            .on("click", (event, d) => {
                event.stopPropagation();
                this.onNodeClick(d);
            })
            .on("dblclick", (event, d) => {
                event.stopPropagation();
                this.toggleCollapse(d.data.uid);
            })
            .on("mouseenter", function(event, d) {
                self._showTooltip(event, d);
            })
            .on("mouseleave", () => {
                this._hideTooltip();
            });
        
        // Node shapes based on type
        this.nodeElements.each(function(d) {
            const nodeType = d.data.type;
            const g = d3.select(this);
            
            if (nodeType === 'Sequence' || nodeType === 'ReactiveSequence' ||
                nodeType === 'Fallback' || nodeType === 'ReactiveFallback' ||
                nodeType === 'Parallel' || nodeType === 'IfThenElse') {
                // Diamond for control nodes
                g.append("polygon")
                    .attr("points", "0,-28 65,0 0,28 -65,0")
                    .attr("class", "node-shape");
            } else if (nodeType.includes('Decorator') || nodeType === 'Inverter' ||
                       nodeType === 'Retry' || nodeType === 'Repeat' ||
                       nodeType === 'ForceSuccess' || nodeType === 'ForceFailure') {
                // Hexagon for decorators
                g.append("polygon")
                    .attr("points", "-55,-16 -28,-26 28,-26 55,-16 55,16 28,26 -28,26 -55,16")
                    .attr("class", "node-shape");
            } else if (d.data.type === 'SubTree') {
                // Rounded rect with thicker border for subtrees
                g.append("rect")
                    .attr("x", -65)
                    .attr("y", -22)
                    .attr("width", 130)
                    .attr("height", 44)
                    .attr("rx", 12)
                    .attr("class", "node-shape");
            } else {
                // Standard rounded rect for actions/conditions
                g.append("rect")
                    .attr("x", -65)
                    .attr("y", -22)
                    .attr("width", 130)
                    .attr("height", 44)
                    .attr("rx", 5)
                    .attr("class", "node-shape");
            }
            
            // Collapse indicator
            if (d.data._collapsed) {
                g.append("text")
                    .attr("x", 55)
                    .attr("y", -15)
                    .attr("class", "subtree-toggle")
                    .text("+");
            }
        });
        
        // Node name labels — use textContent for XSS safety
        this.nodeElements.each(function(d) {
            const g = d3.select(this);
            const name = d.data.displayName || d.data.name || d.data.type;
            const displayText = name.length > 18 ? name.substring(0, 16) + '…' : name;
            
            const textEl = g.append("text")
                .attr("dy", "0.05em")
                .attr("text-anchor", "middle")
                .attr("class", "node-name-text");
            // Using D3's .text() which safely escapes content (no innerHTML)
            textEl.text(displayText);
        });
        
        // Type label (small, below name)
        this.nodeElements.append("text")
            .attr("dy", "1.4em")
            .attr("text-anchor", "middle")
            .attr("class", "node-type-label")
            .text(d => d.data.type);
        
        // Update node count stat
        const statNodes = document.getElementById('stat-nodes');
        if (statNodes) statNodes.textContent = this.d3Root.descendants().length;
        
        // Apply current visual states
        this._applyAllVisualStates();
        
        // Update active path
        this._updateActivePath();
        
        // Fit to screen
        this.fitToScreen();
    }
    
    // =================================================================
    // Status Updates (Groot1 sticky/faded logic)
    // =================================================================
    
    _processBatch(batch) {
        if (!Array.isArray(batch)) return;
        
        this.updateCounter++;
        
        batch.forEach(item => {
            const nodeId = item.node_id;
            const rawStatus = item.status;
            
            // Parse IDLE_FROM_* statuses
            let status = rawStatus;
            let idleFrom = null;
            if (rawStatus === 'IDLE_FROM_SUCCESS') {
                status = 'IDLE';
                idleFrom = 'SUCCESS';
            } else if (rawStatus === 'IDLE_FROM_FAILURE') {
                status = 'IDLE';
                idleFrom = 'FAILURE';
            } else if (rawStatus === 'IDLE_FROM_RUNNING') {
                status = 'IDLE';
                idleFrom = 'RUNNING';
            }
            
            const prevStatus = this.nodeStatuses[nodeId];
            this.nodeStatuses[nodeId] = status;
            
            // Determine visual class with Groot1 sticky/faded logic
            this._applyStatusVisual(nodeId, status, prevStatus, idleFrom);
            
            // Feed & stats (only for real changes)
            if (status !== prevStatus || status === 'RUNNING') {
                if (status === 'SUCCESS' || status === 'FAILURE') {
                    this._addToActionFeed(nodeId, status);
                } else if (status === 'RUNNING' && prevStatus !== 'RUNNING') {
                    this._addToActionFeed(nodeId, status);
                }
            }
        });
        
        // Update side panel
        this._updateRunningNodesList();
        this._updateActivePath();
        this._updateSelectedNodeDetails();
    }
    
    updateNodeStatus(statusData) {
        // Legacy single status update — wrap as batch
        this._processBatch([statusData]);
    }
    
    _applyStatusVisual(nodeId, status, prevStatus, idleFrom) {
        /**
         * Groot1 visual persistence with TEMPORAL IMMUNITY:
         * 
         * When a node enters SUCCESS/FAILURE, the color stays visible
         * for a minimum period even if IDLE arrives immediately after.
         * This prevents fast-flickering when BT ticks at 10Hz.
         * 
         * Timers:
         *   - Vibrant phase: 3.0s (SUCCESS green / FAILURE red, glowing)
         *   - Faded phase:   2.0s (dimmed green / dimmed red)
         *   - Then: IDLE (gray)
         * 
         * Interruption rules:
         *   - RUNNING always interrupts (node is actively executing)
         *   - SUCCESS→FAILURE or FAILURE→SUCCESS interrupts (result changed)
         *   - IDLE / IDLE_FROM_* does NOT interrupt a sticky vibrant/faded state
         *   - SKIPPED always interrupts
         */
        
        const VIBRANT_MS = 3000;  // How long vibrant colors stay
        const FADED_MS   = 2000;  // How long faded colors stay
        
        const existing = this.visualStates[nodeId];
        const currentClass = existing ? existing.class : 'idle';
        
        // --- RUNNING: always takes priority ---
        if (status === 'RUNNING') {
            this._clearTimers(nodeId);
            this.visualStates[nodeId] = { class: 'running', timer1: null, timer2: null };
            this._setNodeClass(nodeId, 'running');
            return;
        }
        
        // --- SKIPPED: always takes priority ---
        if (status === 'SKIPPED') {
            this._clearTimers(nodeId);
            this.visualStates[nodeId] = { class: 'skipped', timer1: null, timer2: null };
            this._setNodeClass(nodeId, 'skipped');
            return;
        }
        
        // --- SUCCESS / FAILURE: start sticky vibrant phase ---
        if (status === 'SUCCESS' || status === 'FAILURE') {
            const cls = status.toLowerCase();
            const fadedCls = `${cls}-faded`;
            
            // If already showing the SAME vibrant color, just refresh the timer
            // If showing a DIFFERENT result (e.g., was success, now failure), interrupt
            const isSameResult = (currentClass === cls || currentClass === fadedCls);
            if (isSameResult && existing && existing.timer1) {
                // Already showing this result — don't restart, let timer continue
                return;
            }
            
            // New result — start fresh sticky cycle
            this._clearTimers(nodeId);
            
            this.visualStates[nodeId] = {
                class: cls,
                timer1: setTimeout(() => {
                    // Phase 2: vibrant → faded
                    if (this.visualStates[nodeId]) {
                        this.visualStates[nodeId].class = fadedCls;
                        this._setNodeClass(nodeId, fadedCls);
                    }
                    
                    if (this.visualStates[nodeId]) {
                        this.visualStates[nodeId].timer2 = setTimeout(() => {
                            // Phase 3: faded → idle
                            if (this.visualStates[nodeId]) {
                                this.visualStates[nodeId].class = 'idle';
                                this._setNodeClass(nodeId, 'idle');
                            }
                        }, FADED_MS);
                    }
                }, VIBRANT_MS),
                timer2: null
            };
            this._setNodeClass(nodeId, cls);
            return;
        }
        
        // --- IDLE / IDLE_FROM_*: respect temporal immunity ---
        if (status === 'IDLE') {
            // KEY: If currently showing a sticky state (vibrant or faded),
            // do NOT interrupt it. Let the timer handle the transition.
            const isSticky = ['success', 'failure', 'success-faded', 'failure-faded'].includes(currentClass);
            if (isSticky) {
                // Temporal immunity: ignore IDLE while sticky is active
                return;
            }
            
            // If we get IDLE_FROM_SUCCESS/FAILURE and we're NOT already sticky,
            // show a brief faded state
            if (idleFrom === 'SUCCESS' || idleFrom === 'FAILURE') {
                const fadedCls = idleFrom === 'SUCCESS' ? 'success-faded' : 'failure-faded';
                
                this._clearTimers(nodeId);
                this.visualStates[nodeId] = {
                    class: fadedCls,
                    timer1: setTimeout(() => {
                        if (this.visualStates[nodeId]) {
                            this.visualStates[nodeId].class = 'idle';
                            this._setNodeClass(nodeId, 'idle');
                        }
                    }, FADED_MS),
                    timer2: null
                };
                this._setNodeClass(nodeId, fadedCls);
                return;
            }
            
            // Plain IDLE — go gray immediately
            this._clearTimers(nodeId);
            this.visualStates[nodeId] = { class: 'idle', timer1: null, timer2: null };
            this._setNodeClass(nodeId, 'idle');
            return;
        }
        
        // --- Unknown status: go idle ---
        this._clearTimers(nodeId);
        this.visualStates[nodeId] = { class: 'idle', timer1: null, timer2: null };
        this._setNodeClass(nodeId, 'idle');
    }
    
    _clearTimers(nodeId) {
        const existing = this.visualStates[nodeId];
        if (existing) {
            if (existing.timer1) clearTimeout(existing.timer1);
            if (existing.timer2) clearTimeout(existing.timer2);
        }
    }
    
    _setNodeClass(nodeId, cls) {
        if (!this.nodeElements) return;
        this.nodeElements
            .filter(d => d.data.uid === nodeId)
            .attr("class", `tree-node ${cls}`);
    }
    
    _applyAllVisualStates() {
        if (!this.nodeElements) return;
        this.nodeElements.attr("class", d => {
            const vs = this.visualStates[d.data.uid];
            const cls = vs ? vs.class : 'idle';
            return `tree-node ${cls}`;
        });
    }
    
    // =================================================================
    // Active Path Highlighting (Groot1)
    // =================================================================
    
    _updateActivePath() {
        if (!this.linkElements || !this.d3Root) return;
        
        // Find all RUNNING node UIDs
        const runningUids = new Set();
        Object.entries(this.nodeStatuses).forEach(([uid, s]) => {
            if (s === 'RUNNING') runningUids.add(Number(uid));
        });
        
        // Build set of ancestor UIDs for all RUNNING nodes
        const activeUids = new Set();
        this.d3Root.descendants().forEach(d => {
            if (runningUids.has(d.data.uid)) {
                // Walk up to root
                let node = d;
                while (node) {
                    activeUids.add(node.data.uid);
                    node = node.parent;
                }
            }
        });
        
        // Apply active-path class to links where both source and target are in activeUids
        this.linkElements
            .classed("active-path", d => 
                activeUids.has(d.source.data.uid) && activeUids.has(d.target.data.uid)
            );
    }
    
    // =================================================================
    // Collapse / Expand SubTrees
    // =================================================================
    
    toggleCollapse(uid) {
        if (this.collapsedSubtrees.has(uid)) {
            this.collapsedSubtrees.delete(uid);
        } else {
            this.collapsedSubtrees.add(uid);
        }
        this.renderTree();
    }
    
    collapseAll() {
        if (!this.treeData || !this.treeData.nodes) return;
        this.treeData.nodes.forEach(n => {
            if ((n.children || []).length > 0 && n.type !== 'BehaviorTree') {
                this.collapsedSubtrees.add(n.uid);
            }
        });
        this.renderTree();
    }
    
    expandAll() {
        this.collapsedSubtrees.clear();
        this.renderTree();
    }
    
    // =================================================================
    // Node Click / Details
    // =================================================================
    
    onNodeClick(nodeData) {
        this.selectedNodeUid = nodeData.data.uid;
        
        // Highlight
        if (this.nodeElements) {
            this.nodeElements.classed("highlighted", false);
            this.nodeElements
                .filter(d => d.data.uid === nodeData.data.uid)
                .classed("highlighted", true);
        }
        
        this._updateSelectedNodeDetails();
    }
    
    _updateSelectedNodeDetails() {
        const container = document.getElementById('node-details');
        if (!container) return;
        
        // Clear safely (no innerHTML)
        container.replaceChildren();
        
        if (this.selectedNodeUid == null || !this.treeData) {
            const empty = document.createElement('div');
            empty.className = 'node-details-empty';
            empty.textContent = 'Click a node to see details';
            container.appendChild(empty);
            return;
        }
        
        // Find node data
        const node = this.treeData.nodes_by_uid?.[this.selectedNodeUid] ||
                     this.treeData.nodes?.find(n => n.uid === this.selectedNodeUid);
        
        if (!node) {
            const empty = document.createElement('div');
            empty.className = 'node-details-empty';
            empty.textContent = 'Node not found';
            container.appendChild(empty);
            return;
        }
        
        const status = this.nodeStatuses[node.uid] || 'IDLE';
        
        const details = [
            { label: 'Name', value: node.name || '—' },
            { label: 'Type', value: node.type },
            { label: 'UID', value: String(node.uid) },
            { label: 'Status', value: status, statusClass: `status-${status.toLowerCase()}` },
            { label: 'Tree', value: node.tree || '—' },
        ];
        
        if (node.subtree_ref) {
            details.push({ label: 'SubTree Ref', value: node.subtree_ref });
        }
        
        details.push({ label: 'Children', value: String((node.children || []).length) });
        
        details.forEach(d => {
            const row = document.createElement('div');
            row.className = 'detail-row';
            
            const labelEl = document.createElement('span');
            labelEl.className = 'detail-label';
            labelEl.textContent = d.label;
            
            const valueEl = document.createElement('span');
            valueEl.className = 'detail-value' + (d.statusClass ? ` ${d.statusClass}` : '');
            valueEl.textContent = d.value;
            valueEl.title = d.value;  // Tooltip for truncated values
            
            row.appendChild(labelEl);
            row.appendChild(valueEl);
            container.appendChild(row);
        });
        
        // Show ports/parameters if available
        const ports = node.ports || {};
        const portKeys = Object.keys(ports);
        if (portKeys.length > 0) {
            const portsHeader = document.createElement('div');
            portsHeader.className = 'detail-row';
            portsHeader.style.marginTop = '0.4rem';
            const headerLabel = document.createElement('span');
            headerLabel.className = 'detail-label';
            headerLabel.textContent = '— Ports —';
            headerLabel.style.color = 'var(--accent)';
            headerLabel.style.fontWeight = '600';
            portsHeader.appendChild(headerLabel);
            container.appendChild(portsHeader);
            
            portKeys.forEach(key => {
                const row = document.createElement('div');
                row.className = 'detail-row';
                
                const labelEl = document.createElement('span');
                labelEl.className = 'detail-label';
                labelEl.textContent = key;
                
                const valueEl = document.createElement('span');
                valueEl.className = 'detail-value';
                valueEl.textContent = ports[key];
                valueEl.title = ports[key];
                
                row.appendChild(labelEl);
                row.appendChild(valueEl);
                container.appendChild(row);
            });
        }
    }
    
    // =================================================================
    // Tooltip
    // =================================================================
    
    _showTooltip(event, d) {
        if (!this.tooltip) return;
        
        const status = this.nodeStatuses[d.data.uid] || 'IDLE';
        const name = d.data.name || d.data.type;
        
        // Build tooltip content safely (no innerHTML with user data)
        this.tooltip.replaceChildren();
        
        const nameEl = document.createElement('div');
        nameEl.className = 'tt-name';
        nameEl.textContent = name;
        this.tooltip.appendChild(nameEl);
        
        const typeEl = document.createElement('div');
        typeEl.className = 'tt-type';
        typeEl.textContent = d.data.type;
        this.tooltip.appendChild(typeEl);
        
        const statusEl = document.createElement('div');
        statusEl.className = 'tt-status';
        statusEl.textContent = status;
        statusEl.style.color = this._statusColor(status);
        this.tooltip.appendChild(statusEl);
        
        // Position tooltip
        this.tooltip.style.left = (event.clientX + 12) + 'px';
        this.tooltip.style.top = (event.clientY - 10) + 'px';
        this.tooltip.classList.add('visible');
    }
    
    _hideTooltip() {
        if (this.tooltip) {
            this.tooltip.classList.remove('visible');
        }
    }
    
    _statusColor(status) {
        const colors = {
            'RUNNING': '#FFC107',
            'SUCCESS': '#4CAF50',
            'FAILURE': '#F44336',
            'IDLE': '#6b7a8d',
            'SKIPPED': '#6a6a70'
        };
        return colors[status] || '#6b7a8d';
    }
    
    // =================================================================
    // Action Feed (anti-spam)
    // =================================================================
    
    _addToActionFeed(nodeId, status) {
        // Only show significant events
        if (!['RUNNING', 'SUCCESS', 'FAILURE'].includes(status)) return;
        
        const node = this.treeData?.nodes_by_uid?.[nodeId] ||
                     this.treeData?.nodes?.find(n => n.uid === nodeId);
        if (!node) return;
        
        // Dedup: skip if same node + same status was the last entry
        const lastStatus = this.lastFeedEntries[nodeId];
        if (lastStatus === status && status === 'RUNNING') return;  // Don't spam RUNNING
        this.lastFeedEntries[nodeId] = status;
        
        const feed = document.getElementById('action-feed');
        if (!feed) return;
        
        // Remove "empty" placeholder
        const emptyEl = feed.querySelector('.empty');
        if (emptyEl) emptyEl.remove();
        
        const timestamp = new Date().toLocaleTimeString('en-US', { 
            hour12: false, 
            hour: '2-digit', 
            minute: '2-digit', 
            second: '2-digit'
        });
        
        const statusEmoji = { 'RUNNING': '▶', 'SUCCESS': '✓', 'FAILURE': '✗' };
        
        const entry = document.createElement('div');
        entry.className = `action-feed-entry status-${status.toLowerCase()}`;
        
        const tsEl = document.createElement('span');
        tsEl.className = 'timestamp';
        tsEl.textContent = timestamp;
        entry.appendChild(tsEl);
        
        const emojiEl = document.createElement('span');
        emojiEl.className = 'emoji';
        emojiEl.textContent = statusEmoji[status] || '•';
        entry.appendChild(emojiEl);
        
        const nameEl = document.createElement('span');
        nameEl.className = 'feed-node-name';
        nameEl.textContent = node.name || node.type;
        entry.appendChild(nameEl);
        
        const statusEl = document.createElement('span');
        statusEl.className = 'status';
        statusEl.textContent = status;
        entry.appendChild(statusEl);
        
        // Insert at top
        feed.insertBefore(entry, feed.firstChild);
        
        // Limit to 30 entries
        while (feed.children.length > 30) {
            feed.removeChild(feed.lastChild);
        }
    }
    
    // =================================================================
    // Running Nodes List
    // =================================================================
    
    _updateRunningNodesList() {
        const runningNodes = Object.entries(this.nodeStatuses)
            .filter(([_, status]) => status === 'RUNNING');
        
        const list = document.getElementById('running-nodes');
        if (!list) return;
        
        // Clear safely
        list.replaceChildren();
        
        const statRunning = document.getElementById('stat-running');
        if (statRunning) statRunning.textContent = runningNodes.length;
        
        if (runningNodes.length === 0) {
            const li = document.createElement('li');
            li.className = 'empty';
            li.textContent = 'No nodes running';
            list.appendChild(li);
        } else {
            runningNodes.forEach(([nodeId]) => {
                const node = this.treeData?.nodes_by_uid?.[nodeId] ||
                             this.treeData?.nodes?.find(n => n.uid === Number(nodeId));
                
                const li = document.createElement('li');
                
                const dot = document.createElement('span');
                dot.className = 'node-status-dot';
                li.appendChild(dot);
                
                const nameSpan = document.createElement('span');
                nameSpan.textContent = node ? (node.name || node.type) : `Node ${nodeId}`;
                li.appendChild(nameSpan);
                
                list.appendChild(li);
            });
        }
    }
    
    // =================================================================
    // Blackboard Viewer
    // =================================================================
    
    fetchBlackboard() {
        fetch('/api/blackboard')
            .then(res => res.json())
            .then(data => {
                this._renderBlackboard(data.blackboard || {});
            })
            .catch(() => {
                // Silently fail — blackboard may not be available
            });
    }
    
    _renderBlackboard(bb) {
        const list = document.getElementById('blackboard-list');
        if (!list) return;
        
        list.replaceChildren();
        
        const keys = Object.keys(bb).sort();
        
        if (keys.length === 0) {
            const li = document.createElement('li');
            li.className = 'blackboard-item';
            const keySpan = document.createElement('span');
            keySpan.className = 'blackboard-key';
            keySpan.textContent = 'No data';
            li.appendChild(keySpan);
            list.appendChild(li);
            return;
        }
        
        keys.forEach(key => {
            const value = bb[key];
            
            const li = document.createElement('li');
            li.className = 'blackboard-item';
            
            const keySpan = document.createElement('span');
            keySpan.className = 'blackboard-key';
            keySpan.textContent = key;
            li.appendChild(keySpan);
            
            const valSpan = document.createElement('span');
            valSpan.className = 'blackboard-value';
            const valStr = String(value);
            valSpan.textContent = valStr;
            
            // Color booleans
            if (valStr === 'true') valSpan.classList.add('val-true');
            else if (valStr === 'false') valSpan.classList.add('val-false');
            
            valSpan.title = valStr;
            li.appendChild(valSpan);
            
            list.appendChild(li);
        });
    }
    
    // =================================================================
    // Stats
    // =================================================================
    
    updateStats() {
        // Update rate
        const now = Date.now();
        const elapsed = (now - this.lastRateCalc) / 1000;
        if (elapsed >= 1) {
            this.currentRate = Math.round(this.updateCounter / elapsed);
            this.updateCounter = 0;
            this.lastRateCalc = now;
        }
        
        const rateEl = document.getElementById('stat-rate');
        if (rateEl) rateEl.textContent = `${this.currentRate} Hz`;
        
        // Uptime
        const uptime = Math.floor((now - this.startTime) / 1000);
        const mins = Math.floor(uptime / 60);
        const secs = uptime % 60;
        const uptimeStr = mins > 0 ? `${mins}m ${secs}s` : `${secs}s`;
        
        const uptimeEl = document.getElementById('uptime');
        if (uptimeEl) uptimeEl.textContent = uptimeStr;
        const wsUptimeEl = document.getElementById('ws-uptime');
        if (wsUptimeEl) wsUptimeEl.textContent = uptimeStr;
    }
    
    // =================================================================
    // Search
    // =================================================================
    
    searchNodes(query) {
        if (!this.nodeElements) return;
        
        if (!query) {
            this.nodeElements.classed("highlighted", d => d.data.uid === this.selectedNodeUid);
            return;
        }
        
        const lowerQuery = query.toLowerCase();
        this.nodeElements.classed("highlighted", d => {
            const name = d.data.name || '';
            const type = d.data.type || '';
            return name.toLowerCase().includes(lowerQuery) || type.toLowerCase().includes(lowerQuery);
        });
    }
    
    // =================================================================
    // Zoom / Pan
    // =================================================================
    
    zoomIn() {
        this.svg.transition().duration(300).call(this.zoom.scaleBy, 1.3);
    }
    
    zoomOut() {
        this.svg.transition().duration(300).call(this.zoom.scaleBy, 0.7);
    }
    
    fitToScreen() {
        const gNode = this.g.node();
        if (!gNode) return;
        
        const bounds = gNode.getBBox();
        const parent = this.svg.node().parentElement;
        if (!parent) return;
        
        const fullWidth = parent.clientWidth;
        const fullHeight = parent.clientHeight;
        const width = bounds.width;
        const height = bounds.height;
        
        if (width === 0 || height === 0) return;
        
        const midX = bounds.x + width / 2;
        const midY = bounds.y + height / 2;
        
        const scale = 0.85 / Math.max(width / fullWidth, height / fullHeight);
        const translate = [fullWidth / 2 - scale * midX, fullHeight / 2 - scale * midY];
        
        this.svg.transition()
            .duration(500)
            .call(this.zoom.transform, d3.zoomIdentity
                .translate(translate[0], translate[1])
                .scale(scale));
    }
    
    resetView() {
        this.svg.transition()
            .duration(500)
            .call(this.zoom.transform, d3.zoomIdentity);
    }
}

// Initialize on page load
document.addEventListener('DOMContentLoaded', () => {
    window.btVisualizer = new BtVisualizer();
});
