import os
import yaml

def merge_yaml_files(input_dir, output_file):
    merged_data = {}

    for filename in os.listdir(input_dir):
        if filename.endswith(".yaml"):
            file_path = os.path.join(input_dir, filename)
            with open(file_path, 'r') as file:
                data = yaml.safe_load(file)
                if data:
                    merged_data.update(data)

    with open(output_file, 'w') as output:
        yaml.dump(merged_data, output)

if __name__ == "__main__":
    input_directory = "/home/abot/abot_ws/src/abot_find/object/"  # 特征文件目录
    output_file = os.path.join(input_directory, "merged_object_db.yaml")  # 合并后的文件路径
    merge_yaml_files(input_directory, output_file)
