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

  // A helper function to create the dialog items
  void createLayout();

  // Event handlers
  void OnKeyDown(wxKeyEvent &event);
  void OnButtonPressed(wxCommandEvent &event);
};
