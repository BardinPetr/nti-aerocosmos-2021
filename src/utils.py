import roslib
import rostopic
from hashlib import blake2b

def encode_topic(data):
    return blake2b(
        data.encode('utf-8') if type(data) is str else data, 
        digest_size=2
    ).digest()

def msg_object_by_topic(topic):
    return roslib.message.get_message_class(rostopic.get_topic_type('/test')[0])

from io import BytesIO

def preprocess_types(data):
    if type(data) is list:
        res = bytearray()
        for i in data:
            res.extend(preprocess_types(i))
    elif 'serialize' in dir(data):
        s = BytesIO()
        data.serialize(s)
        res = s.getvalue()
    elif type(data) is str:
        res = data.encode('utf-8')
    elif type(data) is int:
        res = [data]
    else:
        res = data
    return res