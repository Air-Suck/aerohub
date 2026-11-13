"""Explicit UDP ports and advertised host locators for Docker bridge networking."""
import xml.etree.ElementTree as E

def mapped_profile(path,host,board,domain,index,profile_name='ego_mapped_udp'):
 root=E.Element('profiles',xmlns='http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles')
 tr=E.SubElement(E.SubElement(root,'transport_descriptors'),'transport_descriptor')
 E.SubElement(tr,'transport_id').text='ego_udp';E.SubElement(tr,'type').text='UDPv4'
 for name in ['sendBufferSize','receiveBufferSize']:E.SubElement(tr,name).text='4194304'
 rtps=E.SubElement(E.SubElement(root,'participant',profile_name=profile_name,is_default_profile='true'),'rtps')
 E.SubElement(rtps,'participantID').text=str(index)
 E.SubElement(E.SubElement(rtps,'userTransports'),'transport_id').text='ego_udp'
 E.SubElement(rtps,'useBuiltinTransports').text='false'
 builtin=E.SubElement(rtps,'builtin');meta=7400+250*domain+10+2*index
 for parent,name,port in [(rtps,'defaultUnicastLocatorList',meta+1),(builtin,'metatrafficUnicastLocatorList',meta)]:
  locators=E.SubElement(parent,name)
  for ip in [host,'0.0.0.0']:
   udp=E.SubElement(E.SubElement(locators,'locator'),'udpv4')
   E.SubElement(udp,'address').text=ip;E.SubElement(udp,'port').text=str(port)
 peers=E.SubElement(builtin,'initialPeersList')
 for ip in [host,board,'127.0.0.1']:
  for i in range(32):
   udp=E.SubElement(E.SubElement(peers,'locator'),'udpv4');E.SubElement(udp,'address').text=ip;E.SubElement(udp,'port').text=str(7400+250*domain+10+2*i)
 E.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
