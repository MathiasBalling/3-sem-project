#include "ControllerPanel.h"
#include <wx/wx.h>

// Declare the controller frame (window)
class ControllerFrame : public wxFrame {
public:
  ControllerFrame(const wxString &title);
  ~ControllerFrame();

private:
  void OnAbout(wxCommandEvent &event);
  ControllerPanel *m_controllerPanel;
  wxLog *m_logger;
};

// Declare the controller app (application)
class ControllerApp : public wxApp {
public:
  virtual bool OnInit() {
    ControllerFrame *controllerFrame = new ControllerFrame(wxT("Controller"));
    controllerFrame->Show(true);
    return true;
  };
};

// Start the controller app
IMPLEMENT_APP(ControllerApp)

// Define the controller frame (window)
ControllerFrame::ControllerFrame(const wxString &title)
    : wxFrame(NULL, wxID_ANY, title, wxDefaultPosition, wxSize(600, 400)) {
  // Create a logger window
  m_logger = new wxLogWindow(this, "Audio log", true, false);
  m_logger->SetTimestamp("%H:%M:%S");
  wxLog::SetActiveTarget(m_logger);

  // Create the menu
  wxMenu *menuHelp = new wxMenu;
  menuHelp->Append(wxID_ABOUT);

  wxMenuBar *menuBar = new wxMenuBar;
  menuBar->Append(menuHelp, "&Help");
  SetMenuBar(menuBar);

  // Create the controller panel to capture the key events and add text to the
  // window
  m_controllerPanel = new ControllerPanel(this);

  Bind(wxEVT_MENU, &ControllerFrame::OnAbout, this, wxID_ABOUT);
}

void ControllerFrame::OnAbout(wxCommandEvent &event) {
  (void)event; // To avoid compiler warning about unused parameter
  wxMessageBox(
      "This is a controller, that generates DTMF tones to control a robot.",
      "About the controller", wxOK | wxICON_INFORMATION);
}

ControllerFrame::~ControllerFrame() {
  delete m_controllerPanel;
  delete m_logger;
}
