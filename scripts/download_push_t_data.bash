python ./scripts/download_gdrive_data.py --file_id "1KY1InLurpMvJDRb14L9NlXT_fEsCvVUq&confirm=t" --output_path "./data/push_t_data.zarr.zip"
unzip ./data/push_t_data.zarr.zip -d ./data/push_t_data.zarr
rm -f ./data/push_t_data.zarr.zip
