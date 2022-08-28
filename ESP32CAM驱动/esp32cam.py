from flask import request, Flask, jsonify
import time
app = Flask(__name__)
app.debug = True
@app.route('/test', methods=['POST'])
def add_stu():
    with open(""+str(time.localtime().tm_min)+"."+str(time.localtime().tm_sec)+".jpg","wb") as f:
        f.write(request.data)
    return jsonify({"url":"ok"})
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=80)
