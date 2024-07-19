import datetime

def cmd_resp(type="working", msg="Retransmittion"):
    return {"timestamp":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type":type,
            "msg" : msg}

def state(type="working", workingidx=0, msg="Ready"):
    return {"timestamp":datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "type":type,
            "workingidx":workingidx,
            "msg" : msg}