import rosbag2_py
import csv
import os
import importlib

def export_bag_to_csv(bag_path, output_dir):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)
    
    topics = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topics}
    
    os.makedirs(output_dir, exist_ok=True)
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        
        msg_type_str = topic_type_map[topic].split('/')
        if len(msg_type_str) != 2:
            print(f"Skipping topic with type: {topic_type_map[topic]}")
            continue
        
        # Dynamically import the message type module
        try:
            msg_module = importlib.import_module(f'{msg_type_str[0]}.msg')
            msg_class = getattr(msg_module, msg_type_str[1])
        except (ImportError, AttributeError) as e:
            print(f"Error importing {msg_type_str[0]}.msg.{msg_type_str[1]}: {e}")
            continue
        
        # Deserialize the message data
        try:
            msg = msg_class()
            msg.deserialize(data)
        except Exception as e:
            print(f"Error deserializing message on topic {topic}: {e}")
            continue
        
        csv_file = os.path.join(output_dir, f"{topic.replace('/', '_')}.csv")
        with open(csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if f.tell() == 0:
                # Write header
                writer.writerow(msg.__slots__)
            # Write message data
            writer.writerow([getattr(msg, slot) for slot in msg.__slots__])
        print(f"Exported message on topic {topic}")

if __name__ == '__main__':
    bag_path = '/home/zyj/Documents/GitHub/ROVPS_ws/record/Carto_1'  # Replace with your bag file path
    output_dir = '/home/zyj/Documents/GitHub/ROVPS_ws/script/Carto_1'  # Replace with your desired output directory
    export_bag_to_csv(bag_path, output_dir)
