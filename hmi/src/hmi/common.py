#!/usr/bin/env python
import json
from collections import namedtuple

from hmi_msgs.msg import QueryResult

HMIResult = namedtuple('HMIResult', ['sentence', 'semantics'])


def result_to_ros(result):
    return QueryResult(
        talker_id='',
        sentence=result.sentence,
        semantics=json.dumps(result.semantics)
    )


def result_from_ros(msg):
    return HMIResult(sentence=msg.sentence, semantics=str(json.loads(msg.semantics)))


def trim_string(data, max_length=75, ellipsis='...'):
    l = max_length - len(ellipsis)
    return (data[:l] + ellipsis) if len(data) > l else data


def verify_grammar(grammar, target=None):
    pass


def random_sentence(grammar, target):
    return ""


def parse_sentence(sentence, grammar, target):
    return ""
