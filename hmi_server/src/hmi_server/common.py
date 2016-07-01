#!/usr/bin/env python
import random
import re

import rospy


def trim_string(data, max_length=75, ellipsis='...'):
    l = max_length - len(ellipsis)
    return (data[:l] + ellipsis) if len(data) > l else data


def random_fold_spec(spec, choices):
    """
    From a given spec and choices, return a sentence that is possible

    Args:
        spec: 'My name is <name>'
        choices: {name: ['john', 'jake']}

    Returns:
        'My name is john'
    """
    # first resolve all options
    spec = random_fold_options(spec)

    # choose random choices
    results = {}
    for choice, values in choices.items():
        if len(values):
            value = random.choice(values)
            results[choice] = value
        else:
            rospy.logwarn('No values for choice "%s" in spec "%s"', choice, spec)
            results[choice] = ''

    # Parse the groups in the ending result :)
    return fill_spec_with_results(spec, results)


def get_remaining_choices(spec):
    return re.findall("<([^<>]+)>", spec)


def fill_spec_with_results(spec, results):
    """
    Fill the groups in the spec with the dict results
    """
    remaining_choices = get_remaining_choices(spec)

    for remaining_choice in remaining_choices:
        value = results[remaining_choice]
        spec = spec.replace("<%s>" % remaining_choice, value)

    return spec


def random_fold_options(spec):
    """
    When you have a spec with options, just randomly choose them
    """
    # Pick random group if available
    spec = "(%s)" % spec
    while re.search('\([^\)]+\)', spec):
        options = re.findall('\([^\(\)]+\)', spec)
        for option in options:
            spec = spec.replace(option, random.choice(option[1:-1].split("|")), 1)

    return spec
