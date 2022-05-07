from typing import Mapping

import json
from dataclasses import astuple, dataclass

from grammar_parser.cfgparser import CFGParser
from hmi_msgs.msg import QueryResult


@dataclass(frozen=True)
class HMIResult:
    sentence: str
    semantics: Mapping

    def __iter__(self):
        return iter(astuple(self))


def result_to_ros(result):
    return QueryResult(
        talker_id='',
        sentence=result.sentence,
        semantics=json.dumps(result.semantics)
    )


def result_from_ros(msg):
    return HMIResult(sentence=msg.sentence, semantics=json.loads(msg.semantics))


def trim_string(data, max_length=75, ellipsis='...'):
    l = max_length - len(ellipsis)
    return (data[:l] + ellipsis) if len(data) > l else data


def verify_grammar(grammar, target=None):
    grammar_parser = CFGParser.fromstring(grammar)
    grammar_parser.verify(target)


def random_sentence(grammar, target):
    grammar_parser = CFGParser.fromstring(grammar)
    grammar_parser.verify()
    return grammar_parser.get_random_sentence(target)


def parse_sentence(sentence, grammar, target):
    grammar_parser = CFGParser.fromstring(grammar)
    return grammar_parser.parse(target, sentence)
