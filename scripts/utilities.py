import zmq


def receive_encoded_state(subscriber, wait=False):
    zmq_flag = 0 if wait else zmq.DONTWAIT
    try:
        message = subscriber.recv(zmq_flag)
    except zmq.error.Again:
        return None

    if message:
        return message

