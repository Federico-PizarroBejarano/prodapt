python3 -m prodapt.main --config-dir="checkpoints_final/cube12_5cm" --config-name="cube12_5cm.yaml" mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube12_3"   --config-name="cube12_3.yaml"   mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube12_6"   --config-name="cube12_6.yaml"   mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube12_20"  --config-name="cube12_20.yaml"  mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube12_50"  --config-name="cube12_50.yaml"  mode=evaluate inference.max_steps=1500

python3 -m prodapt.main --config-dir="checkpoints_final/cube1_5cm" --config-name="cube1_5cm.yaml" mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube1_3"   --config-name="cube1_3.yaml"   mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube1_6"   --config-name="cube1_6.yaml"   mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube1_20"  --config-name="cube1_20.yaml"  mode=evaluate inference.max_steps=1500
python3 -m prodapt.main --config-dir="checkpoints_final/cube1_50"  --config-name="cube1_50.yaml"  mode=evaluate inference.max_steps=1500
