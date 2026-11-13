"""Ownership and endpoint contract shared by launch and Adapter.

An empty selection preserves the existing centralized fleet entry. A vehicle
selection owns exactly one execution client, retaining fleet metadata only for
measured peer-body filtering. No alias bridges to the global control endpoints.
"""
import copy
import re


class AdapterScope:
    def __init__(self, config, vehicle=''):
        if not isinstance(vehicle, str):
            raise ValueError('vehicle selection must be a string')
        vehicles = config['vehicles']
        if not vehicles or len(set(vehicles)) != len(vehicles):
            raise ValueError('unique nonempty vehicles required')
        ids = config['planner_ids']
        if (set(ids) != set(vehicles) or len(set(ids.values())) != len(ids)
                or any(type(i) is not int or i < 0 for i in ids.values())):
            raise ValueError('unique nonnegative integer planner ID per vehicle required')
        namespaces = [config['deployment'][v]['namespace'] for v in vehicles]
        if (len(set(namespaces)) != len(namespaces)
                or any(not isinstance(ns, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', ns)
                       for ns in namespaces)):
            raise ValueError('unique unqualified vehicle namespaces required')
        if vehicle and vehicle not in vehicles:
            raise ValueError('selected vehicle is absent from fleet config: '+vehicle)
        self.vehicle = vehicle
        self.fleet = copy.deepcopy(config)
        self.config = copy.deepcopy(config)
        self.namespace = config['deployment'][vehicle]['namespace'] if vehicle else ''
        if vehicle:
            self.config['vehicles'] = [vehicle]
            self.config['planner_ids'] = {vehicle: ids[vehicle]}
            self.config['deployment'] = {vehicle: copy.deepcopy(config['deployment'][vehicle])}

    def api(self, leaf):
        return f'/{self.namespace}/ego/{leaf}' if self.vehicle else f'/ego/{leaf}'

    def planner(self, leaf):
        return f'/{self.namespace}/planner/{leaf}' if self.vehicle else f'/planner/{leaf}'

    def native(self, vehicle, leaf):
        if vehicle not in self.config['vehicles']:
            raise ValueError('native endpoint requested for unowned vehicle')
        return (f'/{self.namespace}/ego/native/{leaf}' if self.vehicle
                else f'/ego/{vehicle}/{leaf}')
