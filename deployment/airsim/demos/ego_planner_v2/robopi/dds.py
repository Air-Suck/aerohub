"""Experimental TCP profiles. Single-pair passed; full multi-process ground test failed."""
import xml.etree.ElementTree as ET


def tcp_profile(path, board_ip, base_port, count=24, listen_port=None, pc_local=None, profile_name='ego_tcp', domain=0):
    root = ET.Element('profiles', xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
    descriptors = ET.SubElement(root, 'transport_descriptors')
    tcp = ET.SubElement(descriptors, 'transport_descriptor')
    ET.SubElement(tcp, 'transport_id').text = 'ego_tcp'
    ET.SubElement(tcp, 'type').text = 'TCPv4'
    tcp_interfaces = ET.SubElement(tcp, 'interfaceWhiteList')
    ET.SubElement(tcp_interfaces, 'address').text = pc_local or board_ip
    ET.SubElement(tcp, 'enable_tcp_nodelay').text = 'true'
    if listen_port is not None:
        ET.SubElement(ET.SubElement(tcp, 'listening_ports'), 'port').text = str(listen_port)
    elif pc_local:
        raise ValueError('PC TCP participants require a distinct fixed listening port')
    udp = ET.SubElement(descriptors, 'transport_descriptor')
    ET.SubElement(udp, 'transport_id').text = 'ego_local_udp'
    ET.SubElement(udp, 'type').text = 'UDPv4'
    interfaces = ET.SubElement(udp, 'interfaceWhiteList')
    local_ips = list(dict.fromkeys([pc_local, '127.0.0.1'])) if pc_local else ['127.0.0.1']
    for ip in local_ips:
        ET.SubElement(interfaces, 'address').text = ip
    rtps = ET.SubElement(ET.SubElement(root, 'participant', profile_name=profile_name, is_default_profile='true'), 'rtps')
    transports = ET.SubElement(rtps, 'userTransports')
    ET.SubElement(transports, 'transport_id').text = 'ego_tcp'
    ET.SubElement(transports, 'transport_id').text = 'ego_local_udp'
    ET.SubElement(rtps, 'useBuiltinTransports').text = 'false'
    builtin = ET.SubElement(rtps, 'builtin')
    peers = ET.SubElement(builtin, 'initialPeersList')
    # Local board participants discover each other on loopback UDP. Only PC
    # participants initiate the cross-host TCP connections through Docker NAT.
    for port in range(base_port, base_port+count) if pc_local else []:
        locator = ET.SubElement(ET.SubElement(peers, 'locator'), 'tcpv4')
        ET.SubElement(locator, 'address').text = board_ip
        ET.SubElement(locator, 'physical_port').text = str(port)
        ET.SubElement(locator, 'port').text = str(7400+250*domain+10)
    # Probe every supported local participant; port-zero defaults cover only
    # a few participant IDs and miss nodes in the complete AS2/EGO launch.
    for ip in local_ips:
        for index in range(32):
            locator = ET.SubElement(ET.SubElement(peers, 'locator'), 'udpv4')
            ET.SubElement(locator, 'address').text = ip
            ET.SubElement(locator, 'port').text = str(7400+250*domain+10+2*index)
    if not len(peers):
        rtps.remove(builtin)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)
