import cereal.messaging as messaging
from selfdrive.controls.lib.dynamic_follow.support import dfProfiles
from common.realtime import sec_since_boot
from common.params import Params


class dfReturn:
  user_profile = None  # stays at user selected profile (사용자가 선택한 프로필에 유지)
  user_profile_text = None  # same as user_profile, but is its text representation (user_profile과 동일하지만 텍스트 표현입니다.)
  model_profile = None  # only changes if user selects auto, is model output (사용자가 자동을 선택한 경우에만 변경되며 모델 출력입니다.)
  model_profile_text = None  # same as model_profile, but is its text representation (model_profile과 동일하지만 텍스트 표현입니다.)
  changed = False  # true if either profile from model or user changes profile (모델의 프로필 또는 사용자가 프로필을 변경하는 경우 true)
  is_auto = False  # true if auto
  last_is_auto = False


class dfManager:
  def __init__(self):
    #self.op_params = opParams()
    self.df_profiles = dfProfiles()
    #self.sm = messaging.SubMaster(['dynamicFollowButton', 'dynamicFollowData'])
    self.sm = messaging.SubMaster(['dynamicFollowData'])
    #self.button_updated = False

    """if self.op_params.get('toyota_distance_btn'):
      # Toyota always resets to stock on car ignition
      self.cur_user_profile = "stock"
    else:
      self.cur_user_profile = self.op_params.get('dynamic_follow').strip().lower()"""

    """if not isinstance(self.cur_user_profile, str) or self.cur_user_profile not in self.df_profiles.to_idx:
      self.cur_user_profile = self.df_profiles.default  # stock (1.45s)
      self.op_params.put('dynamic_follow', self.df_profiles.to_profile[self.cur_user_profile])
    else:
      self.cur_user_profile = self.df_profiles.to_idx[self.cur_user_profile]"""

    self.cur_user_profile = self.df_profiles.to_idx[Params().get("DynamicTRGap", encoding="utf8")]  # String to idx

    self.last_user_profile = self.cur_user_profile

    self.cur_model_profile = 0
    self.alert_duration = 2.0

    self.profile_pred = None
    self.change_time = sec_since_boot()
    self.last_is_auto = False

  @property
  def is_auto(self):
    return self.cur_user_profile == self.df_profiles.auto

  @property
  def can_show_alert(self):
    return sec_since_boot() - self.change_time > self.alert_duration

  def update(self):
    self.sm.update(0)
    df_out = dfReturn()
    """if self.sm.updated['dynamicFollowButton']:
      self.button_updated = True

    if self.button_updated:  # only update when button is first pressed
      self.cur_user_profile = self.sm['dynamicFollowButton'].status"""

    self.cur_user_profile = self.df_profiles.to_idx[Params().get("DynamicTRGap", encoding="utf8")]

    df_out.user_profile = self.cur_user_profile
    df_out.user_profile_text = self.df_profiles.to_profile[df_out.user_profile]

    if self.cur_user_profile != self.last_user_profile:
      #self.op_params.put('dynamic_follow', self.df_profiles.to_profile[df_out.user_profile])  # save current profile for next drive
      #print("if self.cur_user_profile != self.last_user_profile:============== ")
      self.change_time = sec_since_boot()
      self.last_is_auto = False
      df_out.changed = True

    if self.is_auto:  # 자동 모드
      df_out.model_profile = self.sm['dynamicFollowData'].profilePred
      df_out.model_profile_text = self.df_profiles.to_profile[df_out.model_profile]
      df_out.is_auto = True
      df_out.last_is_auto = self.last_is_auto
      self.last_is_auto = True
      if self.cur_model_profile != df_out.model_profile and self.can_show_alert:
        df_out.changed = True  # to hide pred alerts until user-selected auto alert has finished (사용자가 선택한 자동 경고가 완료될 때까지 사전 경고를 숨기려면)
      self.cur_model_profile = df_out.model_profile

    self.last_user_profile = self.cur_user_profile
    return df_out
