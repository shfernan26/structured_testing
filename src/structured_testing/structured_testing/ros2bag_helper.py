import yaml


def getROS2BagData(bag_file):
    last_slash = bag_file.rfind("/")
    bag_dir = bag_file[:last_slash]
    yaml_file = bag_dir + "/metadata.yaml"
    with open(yaml_file, "r") as stream:
        data = yaml.safe_load(stream)
        topics = getTopics(data)
        # If any more paramaters are needed function calls can be added here
    return topics


def getTopics(data):
    topics_set = set()
    for field in data["rosbag2_bagfile_information"]["topics_with_message_count"]:
        topic = field["topic_metadata"]["name"]
        topics_set.add(topic)
    return topics_set
