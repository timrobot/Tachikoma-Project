import requests
import sys
if len(sys.argv) != 2:
  sys.exit()

imgname = sys.argv[1]
r = requests.post("http://localhost:8080/send_image",
  files={"image":open(imgname, "rb")})
print r
