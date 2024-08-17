# client.py

import requests
import json

url = 'http://localhost:5000/todo/api/v1.0/tasks'
#url = 'https://0386-186-29-184-173.ngrok-free.app/todo/api/v1.0/tasks'

response = requests.get(url)

print(str(response))
print('')
print(json.dumps(response.json(), indent=4))