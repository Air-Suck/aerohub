"""Pure lifecycle predicates shared with the offline checks."""
import re


def run_id(value):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,63}', value):
        raise ValueError('Invalid run ID')
    return value


def safe_to_stop(snapshot):
    return (snapshot.get('fresh') is True and snapshot.get('armed') is False
            and snapshot.get('px4_armed') is False
            and snapshot.get('behavior_idle') is True
            and (snapshot.get('ego_state') == 'WAITING'
                 or snapshot.get('stop_confirmed') is True))


def capability_ready(snapshot):
    return (snapshot.get('inputs_ready') is True
            and snapshot.get('interfaces_ready') is True
            and snapshot.get('ego_state') == 'WAITING'
            and snapshot.get('ego_goals') == 0
            and safe_to_stop(snapshot))
