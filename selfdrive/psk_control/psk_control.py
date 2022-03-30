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
GLOBAL_DF_MOD = ntune_scc_get('globalDfMod')
MIN_TR = ntune_scc_get('minTR')

LEAD_SAFE = ntune_scc_get('leadSafe')
RATIO_LEAD_SAFE = ntune_scc_get('ratioLeadSafe')
DURATION_LEAD_SAFE = ntune_scc_get('durationLeadSafe')

CONF_SCC_FILE = '/data/ntune/scc.json'

@app.route('/')
def index():
    return render_template('openpilot_control.html',
                            dynamicFollowParam = DYNAMIC_FOLLOW,
                            globalDfModParam = GLOBAL_DF_MOD,
                            minTRParam = MIN_TR,
                            leadSafeParam = LEAD_SAFE,
                            ratioLeadSafeParam = RATIO_LEAD_SAFE,
                            durationLeadSafeParam = DURATION_LEAD_SAFE
                           )


@app.route('/apply', methods=['GET', 'POST'])
def apply():
    if request.method == 'POST':

        global DYNAMIC_FOLLOW
        DYNAMIC_FOLLOW = request.form['chk_dynamicFollow']
        global GLOBAL_DF_MOD
        GLOBAL_DF_MOD = request.form['global_df_mod']
        global MIN_TR
        MIN_TR = request.form['min_tr']

        global LEAD_SAFE
        LEAD_SAFE = request.form['chk_lead_safe']
        global LEAD_RATIO
        LEAD_RATIO = request.form['lead_safe_ratio']
        global LEAD_DURATION
        LEAD_DURATION = request.form['lead_safe_duration']


        message = '{\n "dynamicFollow": DYNAMIC_FOLLOW,' \
                   '\n "sccGasFactor": SCC_GAS_FACTOR,' \
                   '\n "sccBrakeFactor": SCC_BRAKE_FACTOR,' \
                   '\n "sccCurvatureFactor": SCC_CURVATURE_FACTOR,' \
                   '\n "globalDfMod": GLOBAL_DF_MOD,' \
                   '\n "minTR": MIN_TR,' \
                   '\n "leadSafe": LEAD_SAFE,' \
                   '\n "ratioLeadSafe": LEAD_RATIO,' \
                   '\n "durationLeadSafe": LEAD_DURATION' \
                   '\n }\n'

        #print("before message : ", message)


        message = message.replace('SCC_GAS_FACTOR', str(ntune_scc_get('sccGasFactor')))
        message = message.replace('SCC_BRAKE_FACTOR', str(ntune_scc_get('sccBrakeFactor')))
        message = message.replace('SCC_CURVATURE_FACTOR', str(ntune_scc_get('sccCurvatureFactor')))
        message = message.replace('DYNAMIC_FOLLOW', str(DYNAMIC_FOLLOW))
        message = message.replace('GLOBAL_DF_MOD', str(GLOBAL_DF_MOD))
        message = message.replace('MIN_TR', str(MIN_TR))
        message = message.replace('LEAD_SAFE', str(LEAD_SAFE))
        message = message.replace('RATIO_LEAD_SAFE', str(LEAD_RATIO))
        message = message.replace('DURATION_LEAD_SAFE', str(LEAD_DURATION))

        print("after message : ", message)

        # 파일 저장
        f = open(CONF_SCC_FILE, 'w')
        f.write(message)
        f.close()

        return render_template('openpilot_control.html',
                               dynamicFollowParam=DYNAMIC_FOLLOW,
                               globalDfModParam=GLOBAL_DF_MOD,
                               minTRParam=MIN_TR,
                               leadSafeParam=LEAD_SAFE,
                               ratioLeadSafeParam=RATIO_LEAD_SAFE,
                               durationLeadSafeParam=DURATION_LEAD_SAFE
                               )



def main():
    app.run(host='0.0.0.0', port='7070')

if __name__ == "__main__":
    main()


######
# execute flask
# $ python test_flask.py
######

