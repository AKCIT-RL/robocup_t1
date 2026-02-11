#!/bin/bash
sudo modprobe -v -r xhci_pci
sleep 5
sudo modprobe -v xhci_pci
