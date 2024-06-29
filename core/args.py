import argparse

parser = argparse.ArgumentParser(
    description='Run the BFH Robotic Interface')

server = parser.add_mutually_exclusive_group(required=False)

server.add_argument('-s', '--server_host', 
                    default='localhost', 
                    type=str,
                    help='the address of the grpc robot server; can not be used when local_server is specified.')

server.add_argument('-l', '--local_server', default=argparse.SUPPRESS, help='start a local robot simulator; can not be used when serverhost is specified',action='store_true')

parser.add_argument('-p', '--client_port', default=8080, type=int, help='the port where the nicegui server is exposed on')

args = parser.parse_args()