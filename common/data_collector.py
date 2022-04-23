from common.travis_checker import travis
from selfdrive.swaglog import cloudlog
from common.realtime import sec_since_boot
import threading
import os


class DataCollector:
  def __init__(self, file_path, keys, write_frequency=60, write_threshold=2, log_data=True):
    """
    This class provides an easy way to set up your own custom data collector to gather custom data.
    Parameters:
      file_path (str): The path you want your custom data to be written to.
      keys: (list): A string list containing the names of the values you want to collect.
                    Your data list needs to be in this order.
      write_frequency (int/float): The rate at which to write data in seconds.
      write_threshold (int): The length of the data list we need to collect before considering writing.
    Example:
      data_collector = DataCollector('/data/openpilot/custom_data', ['v_ego', 'a_ego', 'custom_dict'], write_frequency=120)

    이 클래스는 사용자 지정 데이터를 수집하기 위해 사용자 지정 데이터 수집기를 설정하는 쉬운 방법을 제공합니다.
    매개변수:
       file_path(str): 사용자 정의 데이터를 기록할 경로입니다.
       키: (목록): 수집하려는 값의 이름이 포함된 문자열 목록입니다.
                     데이터 목록은 이 순서로 되어 있어야 합니다.
       write_frequency(int/float): 초 단위로 데이터를 쓰는 속도입니다.
       write_threshold(int): 쓰기를 고려하기 전에 수집해야 하는 데이터 목록의 길이입니다.
    예시:
       data_collector = DataCollector('/data/openpilot/custom_data', ['v_ego', 'a_ego', 'custom_dict'], write_frequency=120)
    """

    self.log_data = log_data
    self.file_path = file_path
    self.keys = keys
    self.write_frequency = write_frequency
    self.write_threshold = write_threshold
    self.data = []
    self.last_write_time = sec_since_boot()
    self.thread_running = False
    self._initialize()

  def _initialize(self):  # add keys to top of data file
    if not os.path.exists(self.file_path) and not travis:
      with open(self.file_path, "w") as f:
        f.write('{}\n'.format(self.keys))

  def update(self, sample):
    """
    Appends your sample to a central self.data variable that gets written to your specified file path every n seconds.
    Parameters:
      sample: Can be any type of data. List, dictionary, numbers, strings, etc.
      Or a combination: dictionaries, booleans, and floats in a list
    Continuing from the example above, we assume that the first value is your velocity, and the second
    is your acceleration. IMPORTANT: If your values and keys are not in the same order, you will have trouble figuring
    what data is what when you want to process it later.
    Example:
      data_collector.append([17, 0.5, {'a': 1}])

    n초 마다 지정된 파일 경로에 기록되는 중앙 self.data 변수에 샘플을 추가합니다.
     매개변수:
       샘플: 목록, 사전, 숫자, 문자열 등 모든 유형의 데이터일 수 있습니다.
       또는 조합: 목록의 사전, 부울 및 부동 소수점
     위의 예에서 계속해서 첫 번째 값은 속도이고 두 번째 값은
     중요: 값과 키의 순서가 같지 않으면 계산하는 데 문제가 있습니다.
     어떤 데이터는 나중에 처리하려는 경우입니다.
     예시:
       data_collector.append([17, 0.5, {'a': 1}])
    """

    if self.log_data:
      if len(sample) != len(self.keys):
        raise Exception("You need the same amount of data as you specified in your keys")
      self.data.append(sample)
      self._check_if_can_write()

  def _reset(self, reset_type=None):
    if reset_type in ['data', 'all']:
      self.data = []
    if reset_type in ['time', 'all']:
      self.last_write_time = sec_since_boot()

  def _check_if_can_write(self):
    """
    You shouldn't ever need to call this. It checks if we should write, then calls a thread to do so
    with a copy of the current gathered data. Then it clears the self.data variable so that new data
    can be added and it won't be duplicated in the next write.
    If the thread is still writing by the time of the next write, which shouldn't ever happen unless
    you set a low write frequency, it will skip creating another write thread. If this occurs,
    something is wrong with writing.

    이것을 호출할 필요가 없습니다. 작성해야 하는지 확인한 다음 이를 수행하기 위해 스레드를 호출합니다.
    현재 수집된 데이터의 복사본으로 그런 다음 self.data 변수를 지워서 새 데이터가
    추가할 수 있으며 다음 쓰기에서 중복되지 않습니다.
    쓰레드가 다음 쓰기 시점까지 계속 쓰고 있다면
    쓰기 빈도를 낮게 설정하면 다른 쓰기 스레드 생성을 건너뜁니다.
    쓰기에 문제가 있습니다.
    """

    if (sec_since_boot() - self.last_write_time) >= self.write_frequency and len(self.data) >= self.write_threshold and not travis:
      if not self.thread_running:
        write_thread = threading.Thread(target=self._write, args=(self.data,))
        write_thread.daemon = True
        write_thread.start()
        # self.write(self.data)  # non threaded approach
        self._reset(reset_type='all')
      elif self.write_frequency > 30:
        cloudlog.warning('DataCollector write thread is taking a while to write data.')

  def _write(self, current_data):
    """
    Only write data that has been added so far in background. self.data is still being appended to in
    foreground so in the next write event, new data will be written. This eliminates lag causing openpilot
    critical processes to pause while a lot of data is being written.

    지금까지 추가된 데이터만 백그라운드에서 작성합니다. self.data는 여전히 in에 추가되고 있습니다.
    포그라운드에서 다음 쓰기 이벤트에서 새 데이터가 기록됩니다. 이렇게 하면 지연을 유발하는 오픈 파일럿이 제거됩니다.
    많은 데이터가 기록되는 동안 중요한 프로세스가 일시 중지됩니다.
    """

    self.thread_running = True
    with open(self.file_path, "a") as f:
      f.write('{}\n'.format('\n'.join(map(str, current_data))))  # json takes twice as long to write
    self._reset(reset_type='time')
    self.thread_running = False
