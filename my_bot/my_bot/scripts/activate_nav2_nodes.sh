#!/bin/bash
# Script to activate behavior_server and bt_navigator
# These must be activated in order: behavior_server first, then bt_navigator

echo "=== Activating Nav2 Nodes ==="
echo ""

echo "Step 1: Configuring behavior_server..."
ros2 lifecycle set /behavior_server configure
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to configure behavior_server"
    exit 1
fi
sleep 2

echo "Step 2: Activating behavior_server..."
ros2 lifecycle set /behavior_server activate
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to activate behavior_server"
    exit 1
fi
sleep 2

echo "Step 3: Verifying behavior_server is active..."
STATE=$(ros2 lifecycle get /behavior_server 2>&1 | tail -1)
echo "behavior_server state: $STATE"
if [[ "$STATE" != *"active"* ]]; then
    echo "WARNING: behavior_server is not active. Current state: $STATE"
fi
sleep 2

echo "Step 4: Configuring bt_navigator..."
ros2 lifecycle set /bt_navigator configure
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to configure bt_navigator"
    exit 1
fi
sleep 2

echo "Step 5: Activating bt_navigator..."
ros2 lifecycle set /bt_navigator activate
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to activate bt_navigator"
    exit 1
fi
sleep 2

echo "Step 6: Verifying bt_navigator is active..."
STATE=$(ros2 lifecycle get /bt_navigator 2>&1 | tail -1)
echo "bt_navigator state: $STATE"
if [[ "$STATE" != *"active"* ]]; then
    echo "WARNING: bt_navigator is not active. Current state: $STATE"
    exit 1
fi

echo ""
echo "=== SUCCESS: Both nodes are active! ==="
echo "You can now use 2D Pose Goal in RViz to navigate."
