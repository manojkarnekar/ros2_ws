import docker
client = docker.DockerClient(base_url='unix:///var/run/docker.sock')

try:
    for containers in client.containers.list():
        d = containers.stats(decode=None, stream = False)
        print(d["read"], d["memory_stats"], d["cpu_stats"])
    print("-----------------------------------------------------------------------")

except KeyboardInterrupt:
    print("Closed with CTRL-C")
