python3 -m prodapt.main --config-dir="checkpoints/cube_ours" --config-name="cube_ours.yaml" mode=evaluate inference.interface=ur-driver inference.num_inferences=5
python3 -m prodapt.main --config-dir="checkpoints/cube_3"    --config-name="cube_3.yaml"    mode=evaluate inference.interface=ur-driver inference.num_inferences=5
python3 -m prodapt.main --config-dir="checkpoints/cube_6"    --config-name="cube_6.yaml"    mode=evaluate inference.interface=ur-driver inference.num_inferences=5
python3 -m prodapt.main --config-dir="checkpoints/cube_20"   --config-name="cube_20.yaml"   mode=evaluate inference.interface=ur-driver inference.num_inferences=5
python3 -m prodapt.main --config-dir="checkpoints/cube_50"   --config-name="cube_50.yaml"   mode=evaluate inference.interface=ur-driver inference.num_inferences=5
