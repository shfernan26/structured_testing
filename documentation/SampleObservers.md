
## Single Topic Observers

Single topic Observers are especially helpful for integration testing where individual topics within a system require monitoring/assessment.


### Frequency Observer:
Used to ensure topic is published above an average frequency over the simulation. This is useful to add to critical control signal topics that are known to be slow to compute, ensuring that upstream nodes are getting data at the expected rate.

Example Schema:
```
# Ensures topic "/cmd_vel" is published at 10Hz (or more)
    - name: CmdVelFrequency
      observerClass: FrequencyObserver
      topic: "/cmd_vel"
      msgType: "geometry_msgs.msg.Twist"
      minFreq: 10
```

### Heartbeat Observer:
Used to ensure a topic is published at least once during a simulation. Generally used when you want to make sure the dependencies for a
required topic have been met and it is able to publish something without caring about its contents.

Example Schema:
```
# Make sure that radar topic publishes at least once
    - name: RadarHeartbeat
      observerClass: HeartbeatObserver
      topic: "/radar_tracked_objects"
      msgType: "nav_msgs.msg.Odometry"
```

### In Range Observer:
Ensures that a numeric value is within an acceptable range. Paradigms of { ALWAYS_TRUE , TRUE_ONCE , TRUE_AT_END , TRUE_AT_START }
apply and are used to determine the result.

Example Schema:
```
# Make sure cmd vel in x dir is always within [0, 10]
    - name: CmdVelX
      observerClass: InRangeObserver
      topic: "/cmd_vel"
      msgType: "geometry_msgs.msg.Twist"
      field: "linear.x"
      observerType: ALWAYS_TRUE
      minVal: 0
      maxVal: 10
```

### Min Max Observer:
Similar to the In Range Observer but only checks value in one direction. (Value is always greater than min OR value is always less than max).

Example Schema:
```
# Ensure radar detection in x-direction is always greater than minimum 
    - name: RadarLowerBound
      observerClass: MinMaxObserver
      observerType: ALWAYS_TRUE
      topic: "/radar_tracked_objects"
      msgType: "custom_interface.msg.radar_object"
      field: "detection_in_x"
      value: 2 #in m
      isMin: True #Min Value (if False becomes Max value Observer)
```

### Monotonic Observer:
Used to ensure that a value is always increasing or decreasing throughout the simulation. Useful to determine if fragmented data is coming
in order.

Example Schema:
```
# Ensure sequence number of radar msg is always increasing to confirm data is received in order
    - name: RaderObjectsIDObserver
      observerClass: MinMaxObserver
      observerType: ALWAYS_TRUE
      topic: "/radar_tracked_objects"
      msgType: "nav_msgs.msg.Odometry"
      field: "header.seq"
      isIncreasing: True # If True check for monotonic increase, if false checks for decrease
```

## Multi Topic Observers

Observers can also observe multiple topics simultaneously. This is generally useful for system level testing (e.g. perception output compared to ground truth).

Observers with multiple topics are synchronized using the ROS [ApproximateTimeSynchronizer](http://docs.ros.org/en/lunar/api/message_filters/html/python/index.html). If messages are published within a specified time window apart from each other (currently set to 0.5s) they are grouped together and sent to the Observer. This window size can be modified by changing the `slop` parameter of the `ApproximateTimeSynchronizer` call in [base_observer.py](../structured_testing/observers/base_observer.py)

Existing examples of such Observers include the F1 Score Observer and the OSPA Observer.

### F1 Score Observer
Calculates the F1-Score metric by comparing a target topic (e.g. system detections) to ground truth topic that is being published simultaneously. The error is calculated using the Euclidean distance between the two closest objects. The reference frame assumes 2D positional measurements between the two topics with sub-fields that are named 'dx' (longitude) and 'dy' (latitude). In the example below this corresponds to predictions_array.dx and predictions_array.dy. The required interface format is visualized [here](./F1-and-OSPA-Interface.png).

Example Schema:

```
    - name: SystemF1Score
      observerClass: F1ScoreObserver
      topic:
        - "/prediction_array"      # Predictions topic
        - "/ground_truth_array"     # Ground truth topic
      msgType:
        - "perception.msg.Prediction"       # Prediction message type
        - "perception.msg.GroundTruth"       # Ground truth message type
      field:
        - "predictions_array"     # Prediction field
        - "ground_truth_array"     # Ground Truth field
      num_targets: 2        # Actual number of ground truth objects
      output_type: variable    # Whether output array is of a 'fixed' size or 'variable'
      tol: 2                # Tolerance between GT object and detection
      f1_pass_score: 0.8     # Minimum F1 score that is deemed a PASS (e.g. 80%)
      
```

### OSPA Observer
Calculates the Optimal Sub-Pattern Assignment (OSPA) metric by comparing a target topic (e.g. system detections) to ground truth topic that is being published simultaneously. The reference frame assumes 2D positional measurements between the two topics with sub-fields that are named 'dx' (longitude) and 'dy' (latitude). In the example below this corresponds to predictions_array.dx and predictions_array.dy. The required interface format is visualized [here](./F1-and-OSPA-Interface.png).

Example Schema:
```
    - name: OSPA
      observerClass: OSPAObserver
      topic:
        - "/prediction_array"      # Predictions topic
        - "/ground_truth_array"     # Ground truth topic
      msgType:
        - "perception.msg.Prediction"       # Prediction message type
        - "perception.msg.GroundTruth"       # Ground truth message type
      field:
        - "predictions_array"     # Prediction field
        - "ground_truth_array"     # Ground Truth field
      num_targets: 2
      cut_off: 3      # Cutoff parameter of OSPA metric
      sens: 2     # Sensitivity parameter of OSPA metric
      ospa_pass_score: 3      # Minimum pass score
```