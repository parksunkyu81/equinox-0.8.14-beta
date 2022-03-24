import json
from flask import Flask, render_template
from flask import request
from flask import jsonify, Response
#from cereal import messaging
from selfdrive.ntune import ntune_scc_get

app = Flask(__name__)

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

SCC_GAS_FACTOR = ntune_scc_get('sccGasFactor')
SCC_BRAKE_FACTOR = ntune_scc_get('sccBrakeFactor')
SCC_CURVATURE_FACTOR = ntune_scc_get('sccCurvatureFactor')
DYNAMIC_FOLLOW = ntune_scc_get('dynamicFollow')

CONF_SCC_FILE = '/data/ntune/scc.json'

@app.route('/')
def index():
    return render_template('openpilot_control.html',
                            dynamicFollowParam = DYNAMIC_FOLLOW)


@app.route('/apply', methods=['GET', 'POST'])
def apply():
    if request.method == 'POST':
        global DYNAMIC_FOLLOW
        DYNAMIC_FOLLOW = request.form['chk_dynamicFollow']

        message = '{\n "dynamicFollow": DYNAMIC_FOLLOW,' \
                   '\n "sccGasFactor": SCC_GAS_FACTOR,' \
                   '\n "sccBrakeFactor": SCC_BRAKE_FACTOR,' \
                   '\n "sccCurvatureFactor": SCC_CURVATURE_FACTOR' \
                   '\n }\n'

        #print("message : ", message)


        message = message.replace('SCC_GAS_FACTOR', str(ntune_scc_get('sccGasFactor')))
        message = message.replace('SCC_BRAKE_FACTOR', str(ntune_scc_get('sccBrakeFactor')))
        message = message.replace('SCC_CURVATURE_FACTOR', str(ntune_scc_get('sccCurvatureFactor')))

        message = message.replace('DYNAMIC_FOLLOW', str(DYNAMIC_FOLLOW))


        # 파일 저장
        f = open(CONF_SCC_FILE, 'w')
        f.write(message)
        f.close()

        return render_template('openpilot_control.html',
                                dynamicFollowParam = DYNAMIC_FOLLOW)



def main():
    app.run(host='0.0.0.0', port='7070')

if __name__ == "__main__":
    main()


######
# execute flask
# $ python test_flask.py
######

