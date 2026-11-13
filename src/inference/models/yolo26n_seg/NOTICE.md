The worker and YOLO ROS/Tool integration originate in this user's local
Elastic-Tracker stage2 implementation (GPL-3.0-or-later). The copied integration
files retain that license. Existing Aerohub runtime, TorchBackend and ModelClient
remain unchanged and retain their existing Apache-2.0 notices. The added ROS
message/service field definitions originate in elastic_vision_interfaces.

Ultralytics 8.4.210 and its checkpoint have their upstream AGPL-3.0 / applicable
Enterprise license terms. Checkpoint URL and SHA256 are pinned in artifacts.json.
Weights, converted state_dict, venv, images and raw test logs are ignored artifacts.
No upstream training implementation or planner has been copied into this model.
