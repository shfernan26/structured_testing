import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header

from csv_converter.converter_node import CSVConverter


# Add relevant imports & message 
from common.msg import MobileyeObjectDataMsg, RadarObjectDataMsg, GroundTruth # CHANGE

SPIN_QUEUE = []
PERIOD = 0.01

def map_stock_camera(df_row):
    """
    Populate message based on row of csv (after conversion to dataframe)
    Args:
        df_row: row of csv in pandas dataframe format
    Returns:
        msg: new populated message
    """
    ##################### CHANGE #####################
    
    # call message class and populate message based on needs
    msg = MobileyeObjectDataMsg() 
    for i in range(0,10): 
        obj = str(i+1)

        # Create a Header message and set the timestamp
        timestamp = df_row["Time Stamp"]
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1e9)
        time_msg = Time(seconds=sec, nanoseconds=nsec)
        msg.header.stamp = time_msg.to_msg()

        msg.me_timestamp = df_row["Time Stamp"]
        msg.me_dx[i] = df_row["Detected Dx_"+obj]
        msg.me_dy[i] = df_row["Detected Dy_"+obj]
        # msg.heading_angle[i] = df_row["Heading Angle_"+obj]
        # msg.width[i] = df_row["Width_"+obj]

    return msg

    ##################### CHANGE #####################

def map_stock_radar(df_row):
    """
    Populate message based on row of csv (after conversion to dataframe)
    Args:
        df_row: row of csv in pandas dataframe format
    Returns:
        msg: new populated message
    """
    ##################### CHANGE #####################

    # call message class and populate message based on needs
    msg = RadarObjectDataMsg() 
    for i in range(0,10): 
        obj = str(i+1)

        # Create a Header message and set the timestamp
        timestamp = df_row["Time Stamp"]
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1e9)
        time_msg = Time(seconds=sec, nanoseconds=nsec)
        msg.header.stamp = time_msg.to_msg()

        msg.radar_timestamp = df_row["Time Stamp"] 
        msg.radar_dx[i] = df_row["Detected Dx_"+obj]
        msg.radar_dy[i] = df_row["Detected Dy_"+obj]
        msg.radar_vx[i] = df_row["Detected Vx_"+obj]
        msg.radar_vy[i] = df_row["Detected Vy_"+obj]
        msg.radar_ax[i] = df_row["Detected Ax_"+obj]
        # msg.ay[i] = df_row["Detected Ay_"+obj]
        msg.d_length[i] = df_row["Object Length_"+obj]

    return msg

    ##################### CHANGE #####################

def map_ground_truth(df_row):
    """
    Populate message based on row of csv (after conversion to dataframe)
    Args:
        df_row: row of csv in pandas dataframe format
    Returns:
        msg: new populated message
    """
    ##################### CHANGE #####################

    # call message class and populate message based on needs
    msg = GroundTruth() 
    for i in range(0,2): 
        obj = str(i+1)
        
        # Create a Header message and set the timestamp
        timestamp = df_row["Time Stamp"]
        sec = int(timestamp)
        nsec = int((timestamp - sec) * 1e9)
        time_msg = Time(seconds=sec, nanoseconds=nsec)
        msg.header.stamp = time_msg.to_msg()

        msg.timestamp = df_row["Time Stamp"]
        msg.dx[i] = df_row["Detected Dx_"+obj]
        msg.dy[i] = df_row["Detected Dy_"+obj]
        msg.vx[i] = df_row["Detected Vx_"+obj]
        msg.vy[i] = df_row["Detected Vy_"+obj]
        msg.ax[i] = df_row["Detected Ax_"+obj]
        msg.ay[i] = df_row["Detected Ay_"+obj]


    return msg

    ##################### CHANGE #####################

def main(args=None):
    
    rclpy.init(args=args)

    # CHANGE
    # Add any nodes to create here, remember to rename node and publisher so it is unique
    converter_node1 = CSVConverter(
        node_name="stock_camera",
        csv_path="src/csv_converter/test/stock_front_cam_output.csv",
        mapping_func = map_stock_camera,
        msg_name="MobileyeObjectDataMsg", 
        topic_name="Mobileye_CAN_Rx",
        pub_rate=0.1)
    
    converter_node2 = CSVConverter(
        node_name="stock_radar",
        csv_path="src/csv_converter/test/stock_long_range_radar_output.csv",
        mapping_func = map_stock_radar,
        msg_name="RadarObjectDataMsg", 
        topic_name="Front_Radar_CAN_Rx",
        pub_rate=0.1)

    converter_node3 = CSVConverter(
        node_name="truth_node",
        csv_path="src/csv_converter/test/ground_truth_output.csv",
        mapping_func = map_ground_truth,
        msg_name="GroundTruth", 
        topic_name="Ground_Truth",
        pub_rate=0.1)

    # CHANGE
    # Append your own node here
    SPIN_QUEUE.append(converter_node1)
    SPIN_QUEUE.append(converter_node2)
    SPIN_QUEUE.append(converter_node3)

    while rclpy.ok():
        try:
            for node in SPIN_QUEUE:
                rclpy.spin_once(node)
        except Exception as e:
            print(f"something went wrong in the ROS Loop: {e}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
