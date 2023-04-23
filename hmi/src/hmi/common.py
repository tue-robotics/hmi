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


def result_to_ros(result: HMIResult) -> QueryResult:
    return QueryResult(
        talker_id='',
        sentence=result.sentence,
        semantics=json.dumps(result.semantics)
    )


def result_from_ros(msg: QueryResult) -> HMIResult:
    return HMIResult(sentence=msg.sentence, semantics=json.loads(msg.semantics))


def trim_string(data: str, max_length: int = 75, ellipsis: str = "...") -> str:
    cutoff_length = max_length - len(ellipsis)
    return (data[:cutoff_length] + ellipsis) if len(data) > cutoff_length else data


def verify_grammar(grammar: str, target=None):
    grammar_parser = CFGParser.fromstring(grammar)
    grammar_parser.verify(target)


def random_sentence(grammar: str, target: str) -> str:
    grammar_parser = CFGParser.fromstring(grammar)
    grammar_parser.verify()
    return grammar_parser.get_random_sentence(target)


def parse_sentence(sentence: str, grammar: str, target: str) -> Mapping:
    grammar_parser = CFGParser.fromstring(grammar)
    return grammar_parser.parse(target, sentence)
