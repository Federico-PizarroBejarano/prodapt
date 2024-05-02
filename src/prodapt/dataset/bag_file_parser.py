import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics"
        ).fetchall()
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id}"
        ).fetchall()
        return [
            (
                timestamp / 10.0**9,
                deserialize_message(data, self.topic_msg_message[topic_name]),
            )
            for timestamp, data in rows
        ]
