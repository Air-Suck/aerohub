import json
import os
import urllib.error
import urllib.request
base=os.environ.get('INFERENCE_ENDPOINT', 'http://127.0.0.1:8000').rstrip('/')
with urllib.request.urlopen(base+'/health/ready') as response:
    print('health', response.status, response.read().decode())
request={'request_id':'corrected-hmm-contract','depth':[[20.0]*160 for _ in range(96)],'state':[1.,0.,0.,0.,0.,0.,10.,0.,0.]}
for index in range(3):
    data=json.dumps(request).encode()
    with urllib.request.urlopen(urllib.request.Request(base+'/infer',data=data,headers={'Content-Type':'application/json'})) as response:
        result=json.load(response)
    assert len(result['endstates']) == 15
    assert result['device'].startswith('houmo:')
    print(json.dumps({'iteration':index,'best_index':result['best_index'],'candidates':len(result['endstates']),'device':result['device'],'timings_ms':result['timings_ms']}))
try:
    urllib.request.urlopen(urllib.request.Request(base+'/infer',data=b'{}',headers={'Content-Type':'application/json'}))
except urllib.error.HTTPError as error:
    assert error.code == 422
    print('invalid_request',error.code)
else:
    raise AssertionError('Invalid input was accepted')
