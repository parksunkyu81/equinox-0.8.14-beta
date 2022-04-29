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

KP1 = ntune_scc_get('kp1')
KP2 = ntune_scc_get('kp2')
KP3 = ntune_scc_get('kp3')
KP4 = ntune_scc_get('kp4')
KP5 = ntune_scc_get('kp5')
KP6 = ntune_scc_get('kp6')

KI1 = ntune_scc_get('ki1')
KI2 = ntune_scc_get('ki2')
KI3 = ntune_scc_get('ki3')
KI4 = ntune_scc_get('ki4')
KI5 = ntune_scc_get('ki5')
KI6 = ntune_scc_get('ki6')

TR1 = ntune_scc_get('tr1')
TR2 = ntune_scc_get('tr2')
TR3 = ntune_scc_get('tr3')
TR4 = ntune_scc_get('tr4')
TR5 = ntune_scc_get('tr5')
TR6 = ntune_scc_get('tr6')
TR7 = ntune_scc_get('tr7')
TR8 = ntune_scc_get('tr8')
TR9 = ntune_scc_get('tr9')
TR10 = ntune_scc_get('tr10')


CONF_SCC_FILE = '/data/ntune/scc.json'

@app.route('/')
def index():
    return render_template('openpilot_control.html',
                           kp1Param=KP1,
                           kp2Param=KP2,
                           kp3Param=KP3,
                           kp4Param=KP4,
                           kp5Param=KP5,
                           kp6Param=KP6,
                           ki1Param=KI1,
                           ki2Param=KI2,
                           ki3Param=KI3,
                           ki4Param=KI4,
                           ki5Param=KI5,
                           ki6Param=KI6,
                           tr1Param=TR1,
                           tr2Param=TR2,
                           tr3Param=TR3,
                           tr4Param=TR4,
                           tr5Param=TR5,
                           tr6Param=TR6,
                           tr7Param=TR7,
                           tr8Param=TR8,
                           tr9Param=TR9,
                           tr10Param=TR10
                           )


@app.route('/apply', methods=['GET', 'POST'])
def apply():
    if request.method == 'POST':

        global KP1
        KP1 = request.form['kp1']
        global KP2
        KP2 = request.form['kp2']
        global KP3
        KP3 = request.form['kp3']
        global KP4
        KP4 = request.form['kp4']
        global KP5
        KP5 = request.form['kp5']
        global KP6
        KP6 = request.form['kp6']

        global KI1
        KI1 = request.form['ki1']
        global KI2
        KI2 = request.form['ki2']
        global KI3
        KI3 = request.form['ki3']
        global KI4
        KI4 = request.form['ki4']
        global KI5
        KI5 = request.form['ki5']
        global KI6
        KI6 = request.form['ki6']

        global TR1
        TR1 = request.form['tr1']
        global TR2
        TR2 = request.form['tr2']
        global TR3
        TR3 = request.form['tr3']
        global TR4
        TR4 = request.form['tr4']
        global TR5
        TR5 = request.form['tr5']
        global TR6
        TR6 = request.form['tr6']
        global TR7
        TR7 = request.form['tr7']
        global TR8
        TR8 = request.form['tr8']
        global TR9
        TR9 = request.form['tr9']
        global TR10
        TR10 = request.form['tr10']


        message = '{\n "sccGasFactor": SCC_GAS_FACTOR,' \
                   '\n "sccBrakeFactor": SCC_BRAKE_FACTOR,' \
                   '\n "sccCurvatureFactor": SCC_CURVATURE_FACTOR,' \
                   '\n "kp1": KP1,' \
                   '\n "kp2": KP2,' \
                   '\n "kp3": KP3,' \
                   '\n "kp4": KP4,' \
                   '\n "kp5": KP5,' \
                   '\n "kp6": KP6,' \
                   '\n "ki1": KI1,' \
                   '\n "ki2": KI2,' \
                   '\n "ki3": KI3,' \
                   '\n "ki4": KI4,' \
                   '\n "ki5": KI5,' \
                   '\n "ki6": KI6,' \
                   '\n "tr1": TR1,' \
                   '\n "tr2": TR2,' \
                   '\n "tr3": TR3,' \
                   '\n "tr4": TR4,' \
                   '\n "tr5": TR5,' \
                   '\n "tr6": TR6,' \
                   '\n "tr7": TR7,' \
                   '\n "tr8": TR8,' \
                   '\n "tr9": TR9,' \
                   '\n "tr10": TR10' \
                  '\n }\n'

        print("before message : ", message)


        message = message.replace('SCC_GAS_FACTOR', str(ntune_scc_get('sccGasFactor')))
        message = message.replace('SCC_BRAKE_FACTOR', str(ntune_scc_get('sccBrakeFactor')))
        message = message.replace('SCC_CURVATURE_FACTOR', str(ntune_scc_get('sccCurvatureFactor')))

        message = message.replace('KP1', str(KP1))
        message = message.replace('KP2', str(KP2))
        message = message.replace('KP3', str(KP3))
        message = message.replace('KP4', str(KP4))
        message = message.replace('KP5', str(KP5))
        message = message.replace('KP6', str(KP6))

        message = message.replace('KI1', str(KI1))
        message = message.replace('KI2', str(KI2))
        message = message.replace('KI3', str(KI3))
        message = message.replace('KI4', str(KI4))
        message = message.replace('KI5', str(KI5))
        message = message.replace('KI6', str(KI6))

        message = message.replace('TR1', str(TR1))
        message = message.replace('TR2', str(TR2))
        message = message.replace('TR3', str(TR3))
        message = message.replace('TR4', str(TR4))
        message = message.replace('TR5', str(TR5))
        message = message.replace('TR6', str(TR6))
        message = message.replace('TR7', str(TR7))
        message = message.replace('TR8', str(TR8))
        message = message.replace('TR9', str(TR9))
        message = message.replace('TR10', str(TR10))

        print("after message : ", message)

        # 파일 저장
        f = open(CONF_SCC_FILE, 'w')
        f.write(message)
        f.close()

        return render_template('openpilot_control.html',
                               kp1Param=KP1,
                               kp2Param=KP2,
                               kp3Param=KP3,
                               kp4Param=KP4,
                               kp5Param=KP5,
                               kp6Param=KP6,
                               ki1Param=KI1,
                               ki2Param=KI2,
                               ki3Param=KI3,
                               ki4Param=KI4,
                               ki5Param=KI5,
                               ki6Param=KI6,
                               tr1Param=TR1,
                               tr2Param=TR2,
                               tr3Param=TR3,
                               tr4Param=TR4,
                               tr5Param=TR5,
                               tr6Param=TR6,
                               tr7Param=TR7,
                               tr8Param=TR8,
                               tr9Param=TR9,
                               tr10Param=TR10
                               )



def main():
    app.run(host='0.0.0.0', port='7070')

if __name__ == "__main__":
    main()


######
# execute flask
# $ python test_flask.py
######

