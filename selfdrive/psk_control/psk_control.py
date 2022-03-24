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

CONF_SCC_FILE = '/data/ntune/scc.json'

@app.route('/')
def index():
    return render_template('openpilot_control.html',
                            accelProfileParam = ACCEL_PROFILE,
                            gapParam = DISTANCE_GAP,
                            latParam = LEAD_ACCEL_TAU,
                            leadParam = LEAD_SAFE,
                            leadRatioParam = LEAD_RATIO_SAFE,
                            tFollowParam = T_FOLLOW)


@app.route('/apply', methods=['GET', 'POST'])
def apply():
    if request.method == 'POST':
        global DISTANCE_GAP
        DISTANCE_GAP = request.form['chk_distance']
        global LEAD_ACCEL_TAU
        LEAD_ACCEL_TAU = request.form['lat']
        global ACCEL_PROFILE
        ACCEL_PROFILE = request.form['chk_accelProfile']
        global LEAD_SAFE
        LEAD_SAFE = request.form['chk_lead_safe']
        global LEAD_RATIO_SAFE
        LEAD_RATIO_SAFE = request.form['lead_safe_ratio']
        global T_FOLLOW
        T_FOLLOW = request.form['t_follow']


        message = '{\n "distanceGap": DISTANCE_GAP,' \
                   '\n "accelProfile": ACCEL_PROFILE,' \
                   '\n "t_follow": T_FOLLOW,' \
                   '\n "leadSafe": LEAD_SAFE,' \
                   '\n "leadSafeRatio": LEAD_RATIO_SAFE,' \
                   '\n "sccGasFactor": SCC_GAS_FACTOR,' \
                   '\n "leadAccelTau": LEAD_ACCEL_TAU,' \
                   '\n "sccBrakeFactor": SCC_BRAKE_FACTOR,' \
                   '\n "sccCurvatureFactor": SCC_CURVATURE_FACTOR,' \
                   '\n "longitudinalActuatorDelayLowerBound": LADLB,' \
                   '\n "longitudinalActuatorDelayUpperBound": LADUB' \
                   '\n }\n'

        #print("message : ", message)


        message = message.replace('SCC_GAS_FACTOR', str(ntune_scc_get('sccGasFactor')))
        message = message.replace('SCC_BRAKE_FACTOR', str(ntune_scc_get('sccBrakeFactor')))
        message = message.replace('SCC_CURVATURE_FACTOR', str(ntune_scc_get('sccCurvatureFactor')))


        # 파일 저장
        f = open(CONF_SCC_FILE, 'w')
        f.write(message)
        f.close()

        return render_template('openpilot_control.html',
                                accelProfileParam = ACCEL_PROFILE,
                                gapParam = DISTANCE_GAP,
                                latParam = LEAD_ACCEL_TAU,
                                leadParam = LEAD_SAFE,
                                leadRatioParam = LEAD_RATIO_SAFE,
                                tFollowParam = T_FOLLOW)




def main():
    app.run(host='0.0.0.0', port='7070')

if __name__ == "__main__":
    main()


######
# execute flask
# $ python test_flask.py
######

