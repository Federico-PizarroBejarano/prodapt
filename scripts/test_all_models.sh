# 45 diff iters
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_45_3" --config-name="baseline_med_45_3.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_45_6" --config-name="baseline_med_45_6.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_45_20" --config-name="baseline_med_45_20.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_45_50" --config-name="baseline_med_45_50.yaml" mode=evaluate

# 100 diff iters
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_100_3" --config-name="baseline_med_100_3.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_100_6" --config-name="baseline_med_100_6.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_100_20" --config-name="baseline_med_100_20.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_med_100_50" --config-name="baseline_med_100_50.yaml" mode=evaluate

# Ours
python3 -m prodapt.main --config-dir="checkpoints_sim/medium_45_3_10" --config-name="medium_45_3_10.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/medium_100_3_10" --config-name="medium_100_3_10.yaml" mode=evaluate

# 45 diff iters
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_45_3" --config-name="baseline_45_3.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_45_6" --config-name="baseline_45_6.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_45_20" --config-name="baseline_45_20.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_45_50" --config-name="baseline_45_50.yaml" mode=evaluate

# 100 diff iters
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_100_3" --config-name="baseline_100_3.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_100_6" --config-name="baseline_100_6.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_100_20" --config-name="baseline_100_20.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/baseline_100_50" --config-name="baseline_100_50.yaml" mode=evaluate

# Ours
python3 -m prodapt.main --config-dir="checkpoints_sim/heavy_45_3_10" --config-name="heavy_45_3_10.yaml" mode=evaluate
python3 -m prodapt.main --config-dir="checkpoints_sim/heavy_100_3_10" --config-name="heavy_100_3_10.yaml" mode=evaluate



# for BASE_CONFIG in light medium heavy; do
#     for NUM_DIFFUSION_ITERS in 15 45; do
#         for OBS_HORIZON in 1 3 6; do
#             for ACTION_HORIZON in 3 5 10; do
#                 model="${BASE_CONFIG}_${NUM_DIFFUSION_ITERS}_${OBS_HORIZON}_${ACTION_HORIZON}"
#                 python3 -m prodapt.main --config-dir="checkpoints_finished/${model}" --config-name="${model}.yaml" mode=evaluate
#             done
#         done
#     done
# done

# python3 -m prodapt.main --config-dir="checkpoints/cube_cnn" --config-name=cube_cnn.yaml mode=evaluate
