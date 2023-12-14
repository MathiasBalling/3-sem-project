#pragma once
#include "PAsound.h"
#include <memory>
#include <string>
#include <thread>
#include <wx/wx.h>

class ControllerPanel : public wxPanel {
public:
  ControllerPanel(wxFrame *parent);
  ~ControllerPanel();

private:
  // Sound handling
  std::unique_ptr<PAsound> m_paSound;
  int m_duration = DURATION_MS;
  std::pair<float, float> m_movement = {0, 0};
  std::string m_data = "";

  // A helper function to create the panels content and layout
  void createLayout();

  // Sound input data
  void handleAudioInput() const;
  std::thread m_thread;

  // Event handlers
  wxTimer m_timer;
  bool m_isReadyToPlay = true;
  void OnTimer(wxTimerEvent &event);
  void OnKeyDown(wxKeyEvent &event);
  void OnButtonPressed(wxCommandEvent &event);
};
