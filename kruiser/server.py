
import socket
import threading
import socketserver
import json

class ThreadedTCPRequestHandler(socketserver.BaseRequestHandler):
    def __init__(self, callback, *args, **keys):
        self.callback = callback
        socketserver.BaseRequestHandler.__init__(self, *args, **keys)
        
    def handle(self):
        #rec = self.request.recv(1024)
        #data = str(rec, 'utf8')
        #cur_thread = threading.current_thread()
        #response = bytes("{}: {}".format(cur_thread.name, data), 'utf8')
        #self.request.sendall(rec)
        data = self.request.recv(256)
        self.request.sendall(b'ok')
        self.callback(data) #


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    pass


class Command:
    
    def __init__(self, json_string):
        self.json_data = json.loads('{}')
        try: 
            self.json_data = json.loads(json_string)
            self.cmd = self.json_data["cmd"]
        except:
            print("Could not read JSON")
            self.cmd = None
        
        
    def get(self,key):
        if key in self.json_data:
            return self.json_data[key]
        return None
        
        
    def get_or(self,key,default):
        v = self.get(key)
        if v == None:
            return default
        
        return v