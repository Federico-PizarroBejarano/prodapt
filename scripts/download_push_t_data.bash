# Downloading the data
python3 ./scripts/download_gdrive_data.py --file_id "1KY1InLurpMvJDRb14L9NlXT_fEsCvVUq&confirm=t" --output_path "./data/push_t_data.zarr.zip"
unzip ./data/push_t_data.zarr.zip -d ./data/push_t_data.zarr
rm -f ./data/push_t_data.zarr.zip

# Reorganizing to be consistent with our interface
mkdir ./data/push_t_data.zarr/data/ee_pose
mv ./data/push_t_data.zarr/data/action/* ./data/push_t_data.zarr/data/ee_pose/
mv ./data/push_t_data.zarr/data/action/.z* ./data/push_t_data.zarr/data/ee_pose/
mv ./data/push_t_data.zarr/data/ee_pose/ ./data/push_t_data.zarr/data/action/ee_pose/
cp ./data/push_t_data.zarr/data/.zgroup ./data/push_t_data.zarr/data/action/

mkdir ./data/push_t_data.zarr/data/obs
mv -t ./data/push_t_data.zarr/data/obs ./data/push_t_data.zarr/data/state ./data/push_t_data.zarr/data/n_contacts ./data/push_t_data.zarr/data/keypoint ./data/push_t_data.zarr/data/img
cp ./data/push_t_data.zarr/data/.zgroup ./data/push_t_data.zarr/data/obs/
