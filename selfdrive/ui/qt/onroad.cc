#include "selfdrive/ui/qt/onroad.h"

#include <cmath>

#include <QDebug>
#include <QSound>
#include <QMouseEvent>

#include "selfdrive/common/timing.h"
#include "selfdrive/ui/qt/util.h"
#ifdef ENABLE_MAPS
#include "selfdrive/ui/qt/maps/map.h"
#include "selfdrive/ui/qt/maps/map_helpers.h"
#endif

OnroadWindow::OnroadWindow(QWidget *parent) : QWidget(parent) {
  QVBoxLayout *main_layout  = new QVBoxLayout(this);
  main_layout->setMargin(bdr_s);
  QStackedLayout *stacked_layout = new QStackedLayout;
  stacked_layout->setStackingMode(QStackedLayout::StackAll);
  main_layout->addLayout(stacked_layout);

  QStackedLayout *road_view_layout = new QStackedLayout;
  road_view_layout->setStackingMode(QStackedLayout::StackAll);
  nvg = new NvgWindow(VISION_STREAM_RGB_ROAD, this);
  road_view_layout->addWidget(nvg);

  QWidget * split_wrapper = new QWidget;
  split = new QHBoxLayout(split_wrapper);
  split->setContentsMargins(0, 0, 0, 0);
  split->setSpacing(0);
  split->addLayout(road_view_layout);

  stacked_layout->addWidget(split_wrapper);

  alerts = new OnroadAlerts(this);
  alerts->setAttribute(Qt::WA_TransparentForMouseEvents, true);
  stacked_layout->addWidget(alerts);

  // setup stacking order
  alerts->raise();

  setAttribute(Qt::WA_OpaquePaintEvent);
  QObject::connect(uiState(), &UIState::uiUpdate, this, &OnroadWindow::updateState);
  QObject::connect(uiState(), &UIState::offroadTransition, this, &OnroadWindow::offroadTransition);

  // screen recoder - neokii

  record_timer = std::make_shared<QTimer>();
	QObject::connect(record_timer.get(), &QTimer::timeout, [=]() {
    if(recorder) {
      recorder->update_screen();
    }
  });
	record_timer->start(1000/UI_FREQ);

  QWidget* recorder_widget = new QWidget(this);
  QVBoxLayout * recorder_layout = new QVBoxLayout (recorder_widget);
  recorder_layout->setMargin(35);
  recorder = new ScreenRecoder(this);
  recorder_layout->addWidget(recorder);
  recorder_layout->setAlignment(recorder, Qt::AlignRight | Qt::AlignBottom);

  stacked_layout->addWidget(recorder_widget);
  recorder_widget->raise();
  alerts->raise();

}

void OnroadWindow::updateState(const UIState &s) {
  QColor bgColor = bg_colors[s.status];
  Alert alert = Alert::get(*(s.sm), s.scene.started_frame);
  if (s.sm->updated("controlsState") || !alert.equal({})) {
    if (alert.type == "controlsUnresponsive") {
      bgColor = bg_colors[STATUS_ALERT];
    } else if (alert.type == "controlsUnresponsivePermanent") {
      bgColor = bg_colors[STATUS_DISENGAGED];
    }
    alerts->updateAlert(alert, bgColor);
  }

  if (bg != bgColor) {
    // repaint border
    bg = bgColor;
    update();
  }
}

void OnroadWindow::mouseReleaseEvent(QMouseEvent* e) {

  QPoint endPos = e->pos();
  int dx = endPos.x() - startPos.x();
  int dy = endPos.y() - startPos.y();
  if(std::abs(dx) > 250 || std::abs(dy) > 200) {

    if(std::abs(dx) < std::abs(dy)) {

      if(dy < 0) { // upward
        Params().remove("CalibrationParams");
        Params().remove("LiveParameters");
        QTimer::singleShot(1500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });

        QSound::play("../assets/sounds/reset_calibration.wav");
      }
      else { // downward
        QTimer::singleShot(500, []() {
          Params().putBool("SoftRestartTriggered", true);
        });
      }
    }
    else if(std::abs(dx) > std::abs(dy)) {
      if(dx < 0) { // right to left
        if(recorder)
          recorder->toggle();
      }
      else { // left to right
        if(recorder)
          recorder->toggle();
      }
    }

    return;
  }

  if (map != nullptr) {
    bool sidebarVisible = geometry().x() > 0;
    map->setVisible(!sidebarVisible && !map->isVisible());
  }

  // propagation event to parent(HomeWindow)
  QWidget::mouseReleaseEvent(e);
}

void OnroadWindow::mousePressEvent(QMouseEvent* e) {
  startPos = e->pos();
  //QWidget::mousePressEvent(e);
}

void OnroadWindow::offroadTransition(bool offroad) {
#ifdef ENABLE_MAPS
  if (!offroad) {
    if (map == nullptr && (uiState()->prime_type || !MAPBOX_TOKEN.isEmpty())) {
      MapWindow * m = new MapWindow(get_mapbox_settings());
      map = m;

      QObject::connect(uiState(), &UIState::offroadTransition, m, &MapWindow::offroadTransition);

      m->setFixedWidth(topWidget(this)->width() / 2);
      split->addWidget(m, 0, Qt::AlignRight);

      // Make map visible after adding to split
      m->offroadTransition(offroad);
    }
  }
#endif

  alerts->updateAlert({}, bg);

  // update stream type
  bool wide_cam = Hardware::TICI() && Params().getBool("EnableWideCamera");
  nvg->setStreamType(wide_cam ? VISION_STREAM_RGB_WIDE_ROAD : VISION_STREAM_RGB_ROAD);

  if(offroad && recorder) {
    recorder->stop(false);
  }

}

void OnroadWindow::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.fillRect(rect(), QColor(bg.red(), bg.green(), bg.blue(), 255));
}

// ***** onroad widgets *****

// OnroadAlerts
void OnroadAlerts::updateAlert(const Alert &a, const QColor &color) {
  if (!alert.equal(a) || color != bg) {
    alert = a;
    bg = color;
    update();
  }
}

void OnroadAlerts::paintEvent(QPaintEvent *event) {
  if (alert.size == cereal::ControlsState::AlertSize::NONE) {
    return;
  }
  static std::map<cereal::ControlsState::AlertSize, const int> alert_sizes = {
    {cereal::ControlsState::AlertSize::SMALL, 271},
    {cereal::ControlsState::AlertSize::MID, 420},
    {cereal::ControlsState::AlertSize::FULL, height()},
  };
  int h = alert_sizes[alert.size];
  QRect r = QRect(0, height() - h, width(), h);

  QPainter p(this);

  // draw background + gradient
  p.setPen(Qt::NoPen);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  p.setBrush(QBrush(bg));
  p.drawRect(r);

  QLinearGradient g(0, r.y(), 0, r.bottom());
  g.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.05));
  g.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0.35));

  p.setCompositionMode(QPainter::CompositionMode_DestinationOver);
  p.setBrush(QBrush(g));
  p.fillRect(r, g);
  p.setCompositionMode(QPainter::CompositionMode_SourceOver);

  // text
  const QPoint c = r.center();
  p.setPen(QColor(0xff, 0xff, 0xff));
  p.setRenderHint(QPainter::TextAntialiasing);
  if (alert.size == cereal::ControlsState::AlertSize::SMALL) {
    configFont(p, "Open Sans", 74, "SemiBold");
    p.drawText(r, Qt::AlignCenter, alert.text1);
  } else if (alert.size == cereal::ControlsState::AlertSize::MID) {
    configFont(p, "Open Sans", 88, "Bold");
    p.drawText(QRect(0, c.y() - 125, width(), 150), Qt::AlignHCenter | Qt::AlignTop, alert.text1);
    configFont(p, "Open Sans", 66, "Regular");
    p.drawText(QRect(0, c.y() + 21, width(), 90), Qt::AlignHCenter, alert.text2);
  } else if (alert.size == cereal::ControlsState::AlertSize::FULL) {
    bool l = alert.text1.length() > 15;
    configFont(p, "Open Sans", l ? 132 : 177, "Bold");
    p.drawText(QRect(0, r.y() + (l ? 240 : 270), width(), 600), Qt::AlignHCenter | Qt::TextWordWrap, alert.text1);
    configFont(p, "Open Sans", 88, "Regular");
    p.drawText(QRect(0, r.height() - (l ? 361 : 420), width(), 300), Qt::AlignHCenter | Qt::TextWordWrap, alert.text2);
  }
}

// NvgWindow

NvgWindow::NvgWindow(VisionStreamType type, QWidget* parent) : fps_filter(UI_FREQ, 3, 1. / UI_FREQ), CameraViewWidget("camerad", type, true, parent) {

}

void NvgWindow::initializeGL() {
  CameraViewWidget::initializeGL();
  qInfo() << "OpenGL version:" << QString((const char*)glGetString(GL_VERSION));
  qInfo() << "OpenGL vendor:" << QString((const char*)glGetString(GL_VENDOR));
  qInfo() << "OpenGL renderer:" << QString((const char*)glGetString(GL_RENDERER));
  qInfo() << "OpenGL language version:" << QString((const char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

  prev_draw_t = millis_since_boot();
  setBackgroundColor(bg_colors[STATUS_DISENGAGED]);

  // neokii
  ic_brake = QPixmap("../assets/images/img_brake_disc.png").scaled(img_size, img_size, Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
  ic_autohold_warning = QPixmap("../assets/images/img_autohold_warning.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_autohold_active = QPixmap("../assets/images/img_autohold_active.png").scaled(img_size, img_size, Qt::KeepAspectRatio, Qt::SmoothTransformation);
  ic_nda = QPixmap("../assets/images/img_nda.png");
  ic_hda = QPixmap("../assets/images/img_hda.png");
  ic_acc = QPixmap("../assets/images/img_lat_icon.png");
  ic_lkas = QPixmap("../assets/images/img_long.png");
  ic_tire_pressure = QPixmap("../assets/images/img_tire_pressure.png");
  ic_turn_signal_l = QPixmap("../assets/images/turn_signal_l.png");
  ic_turn_signal_r = QPixmap("../assets/images/turn_signal_r.png");
  ic_satellite = QPixmap("../assets/images/satellite.png");
}

void NvgWindow::updateFrameMat(int w, int h) {
  CameraViewWidget::updateFrameMat(w, h);

  UIState *s = uiState();
  s->fb_w = w;
  s->fb_h = h;
  auto intrinsic_matrix = s->wide_camera ? ecam_intrinsic_matrix : fcam_intrinsic_matrix;
  float zoom = ZOOM / intrinsic_matrix.v[0];
  if (s->wide_camera) {
    zoom *= 0.5;
  }
  // Apply transformation such that video pixel coordinates match video
  // 1) Put (0, 0) in the middle of the video
  // 2) Apply same scaling as video
  // 3) Put (0, 0) in top left corner of video
  s->car_space_transform.reset();
  s->car_space_transform.translate(w / 2, h / 2 + y_offset)
      .scale(zoom, zoom)
      .translate(-intrinsic_matrix.v[2], -intrinsic_matrix.v[5]);
}

void NvgWindow::drawLaneLines(QPainter &painter, const UIState *s) {
  const UIScene &scene = s->scene;
  // lanelines
  for (int i = 0; i < std::size(scene.lane_line_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 1.0, 1.0, std::clamp<float>(scene.lane_line_probs[i], 0.0, 0.7)));
    painter.drawPolygon(scene.lane_line_vertices[i].v, scene.lane_line_vertices[i].cnt);
  }

  // road edges
  for (int i = 0; i < std::size(scene.road_edge_vertices); ++i) {
    painter.setBrush(QColor::fromRgbF(1.0, 0, 0, std::clamp<float>(1.0 - scene.road_edge_stds[i], 0.0, 1.0)));
    painter.drawPolygon(scene.road_edge_vertices[i].v, scene.road_edge_vertices[i].cnt);
  }

  // paint path
  QLinearGradient bg(0, height(), 0, height() / 4);
  if (scene.end_to_end) {
    const auto &orientation = (*s->sm)["modelV2"].getModelV2().getOrientation();
    float orientation_future = 0;
    if (orientation.getZ().size() > 16) {
      orientation_future = std::abs(orientation.getZ()[16]);  // 2.5 seconds
    }
    // straight: 112, in turns: 70
    float curve_hue = fmax(70, 112 - (orientation_future * 420));
    // FIXME: painter.drawPolygon can be slow if hue is not rounded
    curve_hue = int(curve_hue * 100 + 0.5) / 100;

    bg.setColorAt(0.0 / 1.5, QColor::fromHslF(148 / 360., 1.0, 0.5, 1.0));
    bg.setColorAt(0.55 / 1.5, QColor::fromHslF(112 / 360., 1.0, 0.68, 0.8));
    bg.setColorAt(0.9 / 1.5, QColor::fromHslF(curve_hue / 360., 1.0, 0.65, 0.6));
    bg.setColorAt(1.0, QColor::fromHslF(curve_hue / 360., 1.0, 0.65, 0.0));
  } else {
    bg.setColorAt(0, whiteColor(200));
    bg.setColorAt(1, whiteColor(0));
  }
  painter.setBrush(bg);
  painter.drawPolygon(scene.track_vertices.v, scene.track_vertices.cnt);
}

void NvgWindow::drawLead(QPainter &painter, const cereal::ModelDataV2::LeadDataV3::Reader &lead_data, const QPointF &vd, bool is_radar) {
  const float speedBuff = 10.;
  const float leadBuff = 40.;
  const float d_rel = lead_data.getX()[0];
  const float v_rel = lead_data.getV()[0];

  float fillAlpha = 0;
  if (d_rel < leadBuff) {
    fillAlpha = 255 * (1.0 - (d_rel / leadBuff));
    if (v_rel < 0) {
      fillAlpha += 255 * (-1 * (v_rel / speedBuff));
    }
    fillAlpha = (int)(fmin(fillAlpha, 255));
  }

  float sz = std::clamp((25 * 30) / (d_rel / 3 + 30), 15.0f, 30.0f) * 2.35;
  float x = std::clamp((float)vd.x(), 0.f, width() - sz / 2);
  float y = std::fmin(height() - sz * .6, (float)vd.y());

  float g_xo = sz / 5;
  float g_yo = sz / 10;

  QPointF glow[] = {{x + (sz * 1.35) + g_xo, y + sz + g_yo}, {x, y - g_yo}, {x - (sz * 1.35) - g_xo, y + sz + g_yo}};
  painter.setBrush(is_radar ? QColor(86, 121, 216, 255) : QColor(218, 202, 37, 255));
  painter.drawPolygon(glow, std::size(glow));

  // chevron
  QPointF chevron[] = {{x + (sz * 1.25), y + sz}, {x, y}, {x - (sz * 1.25), y + sz}};
  painter.setBrush(redColor(fillAlpha));
  painter.drawPolygon(chevron, std::size(chevron));
}

void NvgWindow::paintGL() {
}

void NvgWindow::paintEvent(QPaintEvent *event) {
  QPainter p;
  p.begin(this);

  p.beginNativePainting();
  CameraViewWidget::paintGL();
  p.endNativePainting();

  UIState *s = uiState();
  if (s->worldObjectsVisible()) {
    drawHud(p);
  }

  p.end();

  double cur_draw_t = millis_since_boot();
  double dt = cur_draw_t - prev_draw_t;
  double fps = fps_filter.update(1. / dt * 1000);
  if (fps < 15) {
    LOGW("slow frame rate: %.2f fps", fps);
  }
  prev_draw_t = cur_draw_t;
}

void NvgWindow::showEvent(QShowEvent *event) {
  CameraViewWidget::showEvent(event);

  auto now = millis_since_boot();
  if(now - last_update_params > 1000*5) {
    last_update_params = now;
    ui_update_params(uiState());
  }

  prev_draw_t = millis_since_boot();
}

void NvgWindow::drawText(QPainter &p, int x, int y, const QString &text, int alpha) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(QColor(0xff, 0xff, 0xff, alpha));
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawTextWithColor(QPainter &p, int x, int y, const QString &text, QColor& color) {
  QFontMetrics fm(p.font());
  QRect init_rect = fm.boundingRect(text);
  QRect real_rect = fm.boundingRect(init_rect, 0, text);
  real_rect.moveCenter({x, y - real_rect.height() / 2});

  p.setPen(color);
  p.drawText(real_rect.x(), real_rect.bottom(), text);
}

void NvgWindow::drawIcon(QPainter &p, int x, int y, QPixmap &img, QBrush bg, float opacity) {
  p.setPen(Qt::NoPen);
  p.setBrush(bg);
  p.drawEllipse(x - radius / 2, y - radius / 2, radius, radius);
  p.setOpacity(opacity);
  p.drawPixmap(x - img_size / 2, y - img_size / 2, img_size, img_size, img);
}

void NvgWindow::drawText2(QPainter &p, int x, int y, int flags, const QString &text, const QColor& color) {
  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(text);
  rect.adjust(-1, -1, 1, 1);
  p.setPen(color);
  p.drawText(QRect(x, y, rect.width()+1, rect.height()), flags, text);
}

void NvgWindow::drawHud(QPainter &p) {

  p.setRenderHint(QPainter::Antialiasing);
  p.setPen(Qt::NoPen);
  p.setOpacity(1.);

  // Header gradient
  QLinearGradient bg(0, header_h - (header_h / 2.5), 0, header_h);
  bg.setColorAt(0, QColor::fromRgbF(0, 0, 0, 0.45));
  bg.setColorAt(1, QColor::fromRgbF(0, 0, 0, 0));
  p.fillRect(0, 0, width(), header_h, bg);

  UIState *s = uiState();

  const SubMaster &sm = *(s->sm);

  drawLaneLines(p, s);

  auto leads = sm["modelV2"].getModelV2().getLeadsV3();
  if (leads[0].getProb() > .5) {
    drawLead(p, leads[0], s->scene.lead_vertices[0], s->scene.lead_radar[0]);
  }
  if (leads[1].getProb() > .5 && (std::abs(leads[1].getX()[0] - leads[0].getX()[0]) > 3.0)) {
    drawLead(p, leads[1], s->scene.lead_vertices[1], s->scene.lead_radar[1]);
  }

  drawMaxSpeed(p);
  drawSpeed(p);
  drawSpeedLimit(p);
  drawRestArea(p);
  drawTurnSignals(p);
  //drawGpsStatus(p);

  if(s->show_debug && width() > 1200)
    drawDebugText(p);

  const auto controls_state = sm["controlsState"].getControlsState();
  const auto device_State = sm["deviceState"].getDeviceState();
  //const auto car_control = sm["carControl"].getCarControl();
  //const auto live_params = sm["liveParameters"].getLiveParameters();

  QColor orangeColor = QColor(52, 197, 66, 255);

  //int x = 700;
  //int y = rect().height() - 15;

  float cpuTemp = 0;
  auto cpuList = device_State.getCpuTempC();

  if (cpuList.size() > 0) {
     for(int i = 0; i < cpuList.size(); i++)
         cpuTemp += cpuList[i];
     cpuTemp /= cpuList.size();
  }

  int cpuUsage = 0;
  auto cpuUsageList = device_State.getCpuUsagePercent();

  if (cpuUsageList.size() > 0) {
     for(int i = 0; i < cpuUsageList.size(); i++)
         cpuUsage += cpuUsageList[i];
     cpuUsage /= cpuUsageList.size();
  }

  QString infoText;
  infoText.sprintf("%s SR(%.2f) PEDAL(%.2f) BAT(%d) HW(CPU %.1f ℃, %d, MEM %d)",
                      s->lat_control.c_str(),
                      controls_state.getSteerRatio(),
                      controls_state.getSccGasFactor(),
                      device_State.getBatteryPercent(),
                      cpuTemp,
                      cpuUsage,
                      device_State.getMemoryUsagePercent()
                      );

  // info
  configFont(p, "Open Sans", 40, "Regular");
  p.setPen(QColor(120, 255, 120, 200));
  p.drawText(rect().left() + 20, rect().height() - 15, infoText);


  drawBottomIcons(p);
}

void NvgWindow::drawBottomIcons(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto car_state = sm["carState"].getCarState();
  auto car_control = sm["carControl"].getCarControl();
  //auto controls_state = sm["controlsState"].getControlsState();

  // 1. 핸들 토크 각도
  int x = 140;
  const int y1 = rect().bottom() - footer_h / 2 - 10;

  QString str;
  QString str2;
  float img_alpha;
  float bg_alpha;
  QColor textColor = QColor(255, 255, 255, 200);

  float steer_angle = car_state.getSteeringAngleDeg();
  float desire_angle = car_control.getActuators().getSteeringAngleDeg();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(80));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  float textSize = 60.f;
  textColor = QColor(255, 255, 255, 200);

  str.sprintf("%.0f°", steer_angle);
  configFont(p, "Open Sans", 45, "Bold");
  textColor = QColor(255, 255, 255, 200);
  drawTextWithColor(p, x, y1-20, str, textColor);

  str2.sprintf("%.0f°", desire_angle);
  configFont(p, "Open Sans", textSize, "Bold");
  textColor = QColor(155, 255, 155, 200);
  drawTextWithColor(p, x, y1+50, str2, textColor);
  p.setOpacity(1.0);

  // 2. VISION DIST
  x = radius / 2 + (bdr_s * 2) + (radius + 50);

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(80));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  textSize = 40.f;
  textColor = QColor(255, 255, 255, 200);

  auto lead_vision = sm["modelV2"].getModelV2().getLeadsV3()[0];
  float vision_dist = lead_vision.getProb() > .5 ? (lead_vision.getX()[0] - 1.5) : 0;

  // Orange Color if less than 15ｍ / Red Color if less than 5ｍ
  if (lead_vision.getProb()) {
    if (vision_dist < 15) {
      textColor = QColor(255, 127, 0, 200);
    } else if (vision_dist < 5) {
      textColor = QColor(255, 0, 0, 200);
    } else {
      textColor = QColor(120, 255, 120, 200);
    }
    str.sprintf("%.1f m, %.1f s", vision_dist, vision_dist * 0.084);
  } else {
    str = "──";
  }

  configFont(p, "Open Sans", 45, "Bold");
  drawText(p, x, y1-20, "DIST", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y1+50, str, textColor);
  p.setOpacity(1.0);

  // 3. LKAS
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 2);
  bool lkas_bool = car_state.getLkasEnable();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(80));
  p.drawEllipse(x - radius / 2, y1 - radius / 2, radius, radius);

  textSize = 60.f;
  textColor = QColor(255, 255, 255, 200);

  if(lkas_bool == true) {
    str = "ON";
    textColor = QColor(120, 255, 120, 200);
  }
  else {
    str = "OFF";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 45, "Bold");
  drawText(p, x, y1-20, "LKAS", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y1+50, str, textColor);
  p.setOpacity(1.0);

  // 4.auto hold
  int autohold = car_state.getAutoHold();
  if(autohold >= 0) {
    x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 3);
    img_alpha = autohold > 0 ? 1.0f : 0.15f;
    bg_alpha = autohold > 0 ? 0.3f : 0.1f;
    drawIcon(p, x, y1, autohold > 1 ? ic_autohold_warning : ic_autohold_active,
            QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
    p.setOpacity(1.0);
  }

  // ================================================================================================================ //

  // 1. SPEED
  x = 140;
  const int y2 = rect().bottom() - (footer_h / 2) - (radius + 50) - 10;

  float cur_speed = std::max(0.0, car_state.getVEgo() * MS_TO_KPH);
  float accel = car_state.getAEgo();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(90));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textSize = 65.f;
  textColor = QColor(255, 255, 255, 200);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    textColor = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    textColor = QColor(255, a, a, 230);
  }

  configFont(p, "Open Sans", 45, "Bold");
  drawText(p, x, y2-20, "SPEED", 200);

  str.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

  // 2. PEDAL
  x = radius / 2 + (bdr_s * 2) + (radius + 50);
  accel = car_control.getActuators().getAccel();

  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(90));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textSize = 60.f;
  textColor = QColor(255, 255, 255, 200);

  if(accel > 0) {
    str = "ACCEL";
    textColor = QColor(120, 255, 120, 200);
  }
  else if(accel == 0.0) {
    str = "--";
    textColor = QColor(255, 185, 15, 200);
  }
  else {
    str = "DECEL";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 45, "Bold");
  drawText(p, x, y2-20, "PEDAL", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

  // 3. ACC
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 2);
  bool acc_bool = car_state.getAdaptiveCruise();
  p.setPen(Qt::NoPen);
  p.setBrush(blackColor(90));
  p.drawEllipse(x - radius / 2, y2 - radius / 2, radius, radius);

  textSize = 60.f;
  textColor = QColor(255, 255, 255, 200);

  if(acc_bool == true) {
    str = "ON";
    textColor = QColor(120, 255, 120, 200);
  }
  else {
    str = "OFF";
    textColor = QColor(254, 32, 32, 200);
  }

  configFont(p, "Open Sans", 45, "Bold");
  drawText(p, x, y2-20, "ACC", 200);

  configFont(p, "Open Sans", textSize, "Bold");
  drawTextWithColor(p, x, y2+50, str, textColor);
  p.setOpacity(1.0);

  // 4. brake
  x = radius / 2 + (bdr_s * 2) + ((radius + 50) * 3);
  bool brake_valid = car_state.getBrakePressed();
  img_alpha = brake_valid ? 1.0f : 0.15f;
  bg_alpha = brake_valid ? 0.3f : 0.1f;
  drawIcon(p, x, y2, ic_brake, QColor(0, 0, 0, (255 * bg_alpha)), img_alpha);
  p.setOpacity(1.0);

}

/*
void NvgWindow::drawMaxSpeed(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto controls_state = sm["controlsState"].getControlsState();
  bool is_metric = s->scene.is_metric;

  // kph
  float applyMaxSpeed = controls_state.getApplyMaxSpeed();
  float cruiseMaxSpeed = controls_state.getCruiseMaxSpeed();
  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  QRect rc(30, 30, 184, 202);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);

  if (is_cruise_set) {
    char str[256];
    if (is_metric)
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(applyMaxSpeed*KM_TO_MILE + 0.5));

    configFont(p, "Open Sans", 45, "Bold");
    drawText(p, rc.center().x(), 100, str, 255);

    if (is_metric)
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed + 0.5));
    else
        snprintf(str, sizeof(str), "%d", (int)(cruiseMaxSpeed*KM_TO_MILE + 0.5));

    configFont(p, "Open Sans", 76, "Bold");
    drawText(p, rc.center().x(), 195, str, 255);
  } else {
    configFont(p, "Open Sans", 48, "sans-semibold");
    drawText(p, rc.center().x(), 100, "MAX", 100);

    configFont(p, "Open Sans", 76, "sans-semibold");
    drawText(p, rc.center().x(), 195, "N/A", 100);
  }
}*/

void NvgWindow::drawMaxSpeed(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  const auto controls_state = sm["controlsState"].getControlsState();

  // kph
  float applyMaxSpeed = controls_state.getApplyMaxSpeed();
  float cruiseMaxSpeed = controls_state.getCruiseMaxSpeed();
  bool is_cruise_set = (cruiseMaxSpeed > 0 && cruiseMaxSpeed < 255);

  QColor yellowColor = QColor(255, 255, 0, 255);
  QColor whiteColor = QColor(255, 255, 255, 255);

  QRect rc(30, 30, 184, 202);
  p.setPen(QPen(QColor(0xff, 0xff, 0xff, 100), 10));
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 20, 20);
  p.setPen(Qt::NoPen);

  QString applyMaxSpeedQstr;
  applyMaxSpeedQstr.sprintf("%.0f", applyMaxSpeed);

  QString cruiseMaxSpeedQstr;
  cruiseMaxSpeedQstr.sprintf("%.0f", cruiseMaxSpeed);

  if (is_cruise_set) {
    configFont(p, "Open Sans", 55, "Bold");
    drawTextWithColor(p, rc.center().x(), 100, applyMaxSpeedQstr, yellowColor);
    configFont(p, "Open Sans", 76, "Bold");
    drawTextWithColor(p, rc.center().x(), 195, cruiseMaxSpeedQstr, whiteColor);
  } else {
    configFont(p, "Open Sans", 55, "sans-semibold");
    drawTextWithColor(p, rc.center().x(), 100, "SET", yellowColor);
    configFont(p, "Open Sans", 76, "sans-semibold");
    drawTextWithColor(p, rc.center().x(), 195, "──", whiteColor);
  }

}

void NvgWindow::drawSpeed(QPainter &p) {
  UIState *s = uiState();
  const SubMaster &sm = *(s->sm);
  float cur_speed = std::max(0.0, sm["carState"].getCarState().getVEgo() * (s->scene.is_metric ? MS_TO_KPH : MS_TO_MPH));
  auto car_state = sm["carState"].getCarState();
  float accel = car_state.getAEgo();

  QColor color = QColor(255, 255, 255, 230);

  if(accel > 0) {
    int a = (int)(255.f - (180.f * (accel/2.f)));
    a = std::min(a, 255);
    a = std::max(a, 80);
    color = QColor(a, a, 255, 230);
  }
  else {
    int a = (int)(255.f - (255.f * (-accel/3.f)));
    a = std::min(a, 255);
    a = std::max(a, 60);
    color = QColor(255, a, a, 230);
  }

  QString speed;
  speed.sprintf("%.0f", cur_speed);
  configFont(p, "Open Sans", 176, "Bold");
  drawTextWithColor(p, rect().center().x(), 230, speed, color);

  configFont(p, "Open Sans", 66, "Regular");
  drawText(p, rect().center().x(), 310, s->scene.is_metric ? "km/h" : "mph", 200);
}

void NvgWindow::drawSpeedLimit(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();

  int activeNDA = roadLimitSpeed.getActive();

  int camLimitSpeed = roadLimitSpeed.getCamLimitSpeed();
  int camLimitSpeedLeftDist = roadLimitSpeed.getCamLimitSpeedLeftDist();

  int sectionLimitSpeed = roadLimitSpeed.getSectionLimitSpeed();
  int sectionLeftDist = roadLimitSpeed.getSectionLeftDist();

  int limit_speed = 0;
  int left_dist = 0;

  if(camLimitSpeed > 0 && camLimitSpeedLeftDist > 0) {
    limit_speed = camLimitSpeed;
    left_dist = camLimitSpeedLeftDist;
  }
  else if(sectionLimitSpeed > 0 && sectionLeftDist > 0) {
    limit_speed = sectionLimitSpeed;
    left_dist = sectionLeftDist;
  }

  if(activeNDA > 0)
  {
      int w = 120;
      int h = 54;
      int x = (width() + (bdr_s*2))/2 - w/2 - bdr_s;
      int y = 40 - bdr_s;

      p.setOpacity(1.f);
      p.drawPixmap(x, y, w, h, activeNDA == 1 ? ic_nda : ic_hda);
  }

  if(limit_speed > 10 && limit_speed < 130)
  {
    int radius_ = 192;

    int x = 30;
    int y = 270;

    p.setPen(Qt::NoPen);
    p.setBrush(QBrush(QColor(255, 0, 0, 255)));
    QRect rect = QRect(x, y, radius_, radius_);
    p.drawEllipse(rect);

    p.setBrush(QBrush(QColor(255, 255, 255, 255)));

    const int tickness = 14;
    rect.adjust(tickness, tickness, -tickness, -tickness);
    p.drawEllipse(rect);

    QString str_limit_speed, str_left_dist;
    str_limit_speed.sprintf("%d", limit_speed);

    if(left_dist >= 1000)
      str_left_dist.sprintf("%.1fkm", left_dist / 1000.f);
    else if(left_dist > 0)
      str_left_dist.sprintf("%dm", left_dist);

    configFont(p, "Open Sans", 80, "Bold");
    p.setPen(QColor(0, 0, 0, 230));
    p.drawText(rect, Qt::AlignCenter, str_limit_speed);

    if(str_left_dist.length() > 0) {
      configFont(p, "Open Sans", 60, "Bold");
      rect.translate(0, radius_/2 + 45);
      rect.adjust(-30, 0, 30, 0);
      p.setPen(QColor(255, 255, 255, 230));
      p.drawText(rect, Qt::AlignCenter, str_left_dist);
    }
  }
  else {
    auto controls_state = sm["controlsState"].getControlsState();
    int sccStockCamAct = (int)controls_state.getSccStockCamAct();
    int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

    if(sccStockCamAct == 2 && sccStockCamStatus == 2) {
      int radius_ = 192;

      int x = 30;
      int y = 270;

      p.setPen(Qt::NoPen);

      p.setBrush(QBrush(QColor(255, 0, 0, 255)));
      QRect rect = QRect(x, y, radius_, radius_);
      p.drawEllipse(rect);

      p.setBrush(QBrush(QColor(255, 255, 255, 255)));

      const int tickness = 14;
      rect.adjust(tickness, tickness, -tickness, -tickness);
      p.drawEllipse(rect);

      configFont(p, "Open Sans", 70, "Bold");
      p.setPen(QColor(0, 0, 0, 230));
      p.drawText(rect, Qt::AlignCenter, "CAM");
    }
  }
}

QPixmap NvgWindow::get_icon_iol_com(const char* key) {
  auto item = ic_oil_com.find(key);
  if(item == ic_oil_com.end()) {
    QString str;
    str.sprintf("../assets/images/oil_com/%s.png", key);

    QPixmap icon = QPixmap(str);
    ic_oil_com[key] = icon;
    return icon;
  }
  else
    return item.value();
}

void NvgWindow::drawRestArea(QPainter &p) {
  if(width() < 1850)
    return;

  const SubMaster &sm = *(uiState()->sm);
  auto roadLimitSpeed = sm["roadLimitSpeed"].getRoadLimitSpeed();
  auto restAreaList = roadLimitSpeed.getRestArea();

  int length = std::size(restAreaList);

  int yPos = 0;
  for(int i = length-1; i >= 0; i--) {
    auto restArea = restAreaList[i];
    auto image = restArea.getImage();
    auto title = restArea.getTitle();
    auto oilPrice = restArea.getOilPrice();
    auto distance = restArea.getDistance();

    if(title.size() > 0 && distance.size() > 0) {
      drawRestAreaItem(p, yPos, image, title, oilPrice, distance, i == 0);
      yPos += 200 + 25;
    }
  }
}

void NvgWindow::drawRestAreaItem(QPainter &p, int yPos, capnp::Text::Reader image, capnp::Text::Reader title,
        capnp::Text::Reader oilPrice, capnp::Text::Reader distance, bool lastItem) {

  int mx = 20;
  int my = 5;

  int box_width = Hardware::TICI() ? 580 : 510;
  int box_height = 200;

  int icon_size = 70;

  //QRect rc(30, 30, 184, 202); // MAX box
  QRect rc(184+30+30, 30 + yPos, box_width, box_height);
  p.setBrush(QColor(0, 0, 0, 100));
  p.drawRoundedRect(rc, 5, 5);

  if(lastItem)
    p.setPen(QColor(255, 255, 255, 200));
  else
    p.setPen(QColor(255, 255, 255, 150));

  int x = rc.left() + mx;
  int y = rc.top() + my;

  configFont(p, "Open Sans", 60, "Bold");
  p.drawText(x, y+60+5, title.cStr());

  QPixmap icon = get_icon_iol_com(image.cStr());
  p.drawPixmap(x, y + box_height/2 + 5, icon_size, icon_size, icon);

  configFont(p, "Open Sans", 50, "Bold");
  p.drawText(x + icon_size + 15, y + box_height/2 + 50 + 5, oilPrice.cStr());

  configFont(p, "Open Sans", 60, "Bold");

  QFontMetrics fm(p.font());
  QRect rect = fm.boundingRect(distance.cStr());

  p.drawText(rc.left()+rc.width()-rect.width()-mx-5, y + box_height/2 + 60, distance.cStr());
}

void NvgWindow::drawTurnSignals(QPainter &p) {
  static int blink_index = 0;
  static int blink_wait = 0;
  static double prev_ts = 0.0;

  if(blink_wait > 0) {
    blink_wait--;
    blink_index = 0;
  }
  else {
    const SubMaster &sm = *(uiState()->sm);
    auto car_state = sm["carState"].getCarState();
    bool left_on = car_state.getLeftBlinker();
    bool right_on = car_state.getRightBlinker();

    const float img_alpha = 0.8f;
    const int fb_w = width() / 2 - 200;
    const int center_x = width() / 2;
    const int w = fb_w / 25;
    const int h = 160;
    const int gap = fb_w / 25;
    const int margin = (int)(fb_w / 3.8f);
    const int base_y = (height() - h) / 2;
    const int draw_count = 8;

    int x = center_x;
    int y = base_y;

    if(left_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        p.setOpacity(alpha);
        float factor = (float)draw_count / (i + draw_count);
        p.drawPixmap(x - w - margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_l);
        x -= gap + w;
      }
    }

    x = center_x;
    if(right_on) {
      for(int i = 0; i < draw_count; i++) {
        float alpha = img_alpha;
        int d = std::abs(blink_index - i);
        if(d > 0)
          alpha /= d*2;

        float factor = (float)draw_count / (i + draw_count);
        p.setOpacity(alpha);
        p.drawPixmap(x + margin, y + (h-h*factor)/2, w*factor, h*factor, ic_turn_signal_r);
        x += gap + w;
      }
    }

    if(left_on || right_on) {

      double now = millis_since_boot();
      if(now - prev_ts > 900/UI_FREQ) {
        prev_ts = now;
        blink_index++;
      }

      if(blink_index >= draw_count) {
        blink_index = draw_count - 1;
        blink_wait = UI_FREQ/4;
      }
    }
    else {
      blink_index = 0;
    }
  }

  p.setOpacity(1.);
}

/*
void NvgWindow::drawGpsStatus(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  auto gps = sm["gpsLocationExternal"].getGpsLocationExternal();
  float accuracy = gps.getAccuracy();
  if(accuracy < 0.01f || accuracy > 20.f)
    return;

  int w = 120;
  int h = 100;
  int x = width() - w - 30;
  int y = 30;

  p.setOpacity(0.8);
  p.drawPixmap(x, y, w, h, ic_satellite);

  configFont(p, "Open Sans", 40, "Bold");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  QRect rect = QRect(x, y + h + 10, w, 40);
  rect.adjust(-30, 0, 30, 0);

  QString str;
  str.sprintf("%.1fm", accuracy);
  p.drawText(rect, Qt::AlignHCenter, str);
  p.setOpacity(1.);
}*/

void NvgWindow::drawDebugText(QPainter &p) {
  const SubMaster &sm = *(uiState()->sm);
  QString str, temp;

  int y = 80;
  const int height = 60;

  const int text_x = width()/2 + 200;

  auto controls_state = sm["controlsState"].getControlsState();
  auto car_control = sm["carControl"].getCarControl();
  auto car_state = sm["carState"].getCarState();

  //float applyAccel = controls_state.getApplyAccel();
  //float aReqValue = controls_state.getAReqValue();
  //float aReqValueMin = controls_state.getAReqValueMin();
  //float aReqValueMax = controls_state.getAReqValueMax();

  //int sccStockCamAct = (int)controls_state.getSccStockCamAct();
  //int sccStockCamStatus = (int)controls_state.getSccStockCamStatus();

  float vEgo = car_state.getVEgo();
  float vEgoRaw = car_state.getVEgoRaw();
  int longControlState = (int)controls_state.getLongControlState();
  float vPid = controls_state.getVPid();
  float upAccelCmd = controls_state.getUpAccelCmd();
  float uiAccelCmd = controls_state.getUiAccelCmd();
  float ufAccelCmd = controls_state.getUfAccelCmd();
  float accel = car_control.getActuators().getAccel();

  const char* long_state[] = {"off", "pid", "stopping", "starting"};

  configFont(p, "Open Sans", 50, "Regular");
  p.setPen(QColor(255, 255, 255, 200));
  p.setRenderHint(QPainter::TextAntialiasing);

  str.sprintf("State: %s\n", long_state[longControlState]);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vEgo: %.2f/%.2f\n", vEgo*3.6f, vEgoRaw*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("vPid: %.2f/%.2f\n", vPid, vPid*3.6f);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("P: %.3f\n", upAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("I: %.3f\n", uiAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("F: %.3f\n", ufAccelCmd);
  p.drawText(text_x, y, str);

  y += height;
  str.sprintf("Accel: %.3f\n", accel);
  p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("Apply: %.3f, Stock: %.3f\n", applyAccel, aReqValue);
  //p.drawText(text_x, y, str);

  //y += height;
  //str.sprintf("%.3f (%.3f/%.3f)\n", aReqValue, aReqValueMin, aReqValueMax);
  //p.drawText(text_x, y, str);

  //auto lead_radar = sm["radarState"].getRadarState().getLeadOne();
  //auto lead_one = sm["modelV2"].getModelV2().getLeadsV3()[0];

  //float radar_dist = lead_radar.getStatus() && lead_radar.getRadar() ? lead_radar.getDRel() : 0;
  //float vision_dist = lead_one.getProb() > .5 ? (lead_one.getX()[0] - 1.5) : 0;

  //y += height;
  //str.sprintf("Lead: %.1f/%.1f/%.1f\n", radar_dist, vision_dist, (radar_dist - vision_dist));
  //p.drawText(text_x, y, str);
}
