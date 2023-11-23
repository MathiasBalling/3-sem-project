#pragma once
#include "PAsound.h"
#include <memory>
#include <vector>
#include <wx/wx.h>

class ControllerPanel : public wxPanel {
public:
  ControllerPanel(wxFrame *parent);
  ~ControllerPanel();

private:
  std::unique_ptr<PAsound> m_paSound;
  wxTextCtrl *m_textCtrl;
  int m_duration = DURATION_MS;
  std::pair<float, float> m_movement = {0, 0};

  // A helper function to create the panels content and layout
  void createLayout();

  // Event handlers
  void OnKeyDown(wxKeyEvent &event);
  void OnButtonPressed(wxCommandEvent &event);
};
