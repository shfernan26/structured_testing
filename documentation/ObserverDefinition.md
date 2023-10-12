# Structured Testing Observer Schema

Observers are developed to monitor a simulation and assess its performance. They subscribe to a topic and watch each message published. At the end of a simulation run, the Observer is expected to return a PASS/FAIL result.

## Using Existing Observers

There are many observers already built into the structured testing repo that can be used [out of the box](SampleObservers.md) using _.sim_ files.

### Schema:

Observers are added to a .sim file by putting them in a list under the _observers_ section. Each type of Observer has 5 common fields. Additional fields are specific to Observer implementation.

`name:` Name of the Observer, can be anything used to associate results file with Observer.

`observerClass:` Class defining the functionality of the Observer, has to be present in the _observers_ directory of structured testing.

`observerType:` Type of Observer must be one of { `ALWAYS_TRUE` , `TRUE_ONCE` , `TRUE_AT_END` , `TRUE_AT_START` }. NOTE: Not required for all observers (i.e Frequency and Heartbeat do not need this field).

`topic:` Name of topic to subscribe to (using the global namespace).

`msgType:` Data type of msg, in the format it would be imported in Python and not C++ (i.e use std_msgs.msg.Float32 and NOT std_msgs/Float32). 

`field:` Subfield of the msg that is being consumed indexed through a `.` operator (see examples [here](SampleObservers.md)).


## Adding New Observers:

The observer framework is designed to support rapid development of new custom Observers targeted to specific use cases. In order to
make a new Observer, define a new class in the _observers_ directory of structured testing. The class should inherit from the [BaseObserver](./../structured_testing/observers/base_observer.py).

### Defining an Initializer:

An initializer should take the following form. Ensure that the super initializer is called.
```
    def __init__(self, *, name, topic, msgType, observerType, customArg1 customArg2, ... , **kwargs):
        super().__init__(name, [topic], [msgType], observerType)
```

If the Observer takes in multiple topics and message types remove the square brackets from the super initilaizer arguments.
```
    def __init__(self, *, name, topic, msgType, observerType, customArg1, customArg2, ... , **kwargs):
        super().__init__(name, topic, msgType, observerType)
```

Custom arguments specific to the new Observer (`customArg1` , `customArg2` above) can be specified in the _.sim_ file keyed exactly the same as the variable name in the function signature of the initializer.

### Define consumeMsg():
This function is called every time a msg is published, and assures the data is parsed and understood within the function. 

### Define getResult():
This function is called once at the end of the simulation and is expected to return a boolean that corresponds to the PASS/FAIL of a test. 

### Define metadataDict():
This function is called once at the end of the simulation and is expected to return a dictionary, the contents of which are dumped to
the _.simresults_ file. It is useful to use this function to specify more information about the final result other than just PASS/FAIL.