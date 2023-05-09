import rclpy
from rclpy.node import Node
import pandas as pd

# Add relevant imports & message 
from common.msg import * 

class CSVConverter(Node):
    """
    Takes in csv data, converts it to dataframe and then publishes contents row
    by row at specified publish rate. 
    Args:
        node_name (string): Name of new node (string). Each one should be distinct
        csv_path (string): Path to csv file
        map (function): Custom mapping function defined in run_converter.py that maps csv (dataframe) row to message field
        msg_name (string): Name of messsage WITHOUT .msg 
        topic_name (string): Name of publisher
        pub_rate (float): Publish rate (ex. 0.1 = new message published every 0.1s)
    Returns:
        msg: New  node instance
    
    """

    def __init__(self, node_name, csv_path, mapping_func, msg_name, topic_name, pub_rate=0.1):
        super().__init__(node_name)
        self.df = pd.read_csv(csv_path)
        self.mapping_func = mapping_func
        self.publisher_ = self.create_publisher(
            eval(msg_name),
            topic_name,
             10)
        row_count, col_count = self.df.shape
        self.row_count = row_count
    
        self.current_row = 0
        self.timer = self.create_timer(
            pub_rate, self.timer_callback)
        

    def timer_callback(self):
        """
        Callback called at specified interval to publish csv row as ROS topic
        Args:
            None
        Returns:
            None
        """

        # shut down ROS node at end of csv
        if self.current_row > self.row_count-1:
            rclpy.shutdown()
            print("Node shutdown")

        df_row = self.df.iloc[self.current_row]

        mapped_msg = self.mapping_func(df_row)

        self.publisher_.publish(mapped_msg)
        self.current_row += 1
