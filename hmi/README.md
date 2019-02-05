# HMI: Human Machine Interface

[![Build Status](https://travis-ci.org/tue-robotics/hmi.svg?branch=master)](https://travis-ci.org/tue-robotics/hmi)

The HMI allows a robot to ask questions and get answers back, simple as that. 

Questions are posed in the form of a context-feature grammar, 
to be used by the [grammar_parser](https://github.com/tue-robotics/grammar_parser/), and a target in that grammar.

Answering a question has two sides: a client who asks the question and a server that answers a question.

## AbstractHMIServer
The `AbstractHMIServer` must be subclassed with the 
```_determine_answer(self, description, grammar, target, is_preempt_requested)``` overridden. 

The result should be a ```HMIResult(sentence, semantics)```, where the semantics is a (nested) dictionary,
as returned by the grammar parser (e.g. via ```semantics = grammar_parser.parse(target, sentence)```)

There are various examples used for testing in 
[scripts/test/mocks](https://github.com/tue-robotics/hmi/tree/master/hmi/test/mocks)

## Client
The client can ask questions by using a ```hmi.Client``` objects and 
calling its ```query(description, grammar, target, timeout=10)``` method, for example:
```python
from hmi import Client
from hmi_msgs.msg import QueryAction
import actionlib

client = Client(simple_action_client=actionlib.SimpleActionClient('/robot/hmi', QueryAction))
result = client.query("What's it gonna be boy, yes or no?", 
                      'T -> yes | no', 'T')
assert result.sentence == 'yes' or result.sentence == 'no'
```

Note that the Robot-objects from `robot_skills` already offer a HMI client via `robot.api` 
and thus the `robot_smach_states` do usually not have to instantiate a Client themselves.  

## Behind the scenes
The way the client and server work together is not direct; there is something more happening.
There is more than one modality to interface with a robot. 
You may talk with it, use text messages, QR-codes, a touch screen, whatever.
There are useful test clients as well, that reply a random fitting answer back or replay a conversation. 

The HMI system allows to pose the same question to multiple servers that might answer the question, 
each with their own interfacing modality. 

So, the `actionlib.ActionServer` behind `'/robot/hmi'` distributes the goal (question) from the `hmi.Client`
to multiple servers. 
The node that does this is the (incorrectly named maybe) 
[`multi_client`](https://github.com/tue-robotics/hmi/blob/master/hmi/scripts/multi_client)

When you pass it a goal, it tries to find all action server capable of answering questions 
(i.e. it finds implementers of `QueryAction`)
and then passes the same question to all of those. 

When an answer is received from any one, the others are cancelled and 
the first answer is replied back to the client.
