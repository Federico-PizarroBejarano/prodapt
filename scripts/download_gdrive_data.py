import argparse
import os
import gdown


def download_gdrive_data(file_id, output_path):
    if not os.path.exists(os.path.dirname(output_path)):
        os.makedirs(os.path.dirname(output_path))
    gdown.download(id=file_id, output=output_path, quiet=False)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Download file from google drive")
    parser.add_argument("--file_id", type=str, help="File id of the google drive file")
    parser.add_argument("--output_path", type=str, help="Output path to save the file")
    args = parser.parse_args()
    download_gdrive_data(args.file_id, args.output_path)
