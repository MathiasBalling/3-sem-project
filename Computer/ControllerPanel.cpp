#include "ControllerPanel.h"
#include "consts.h"

ControllerPanel::ControllerPanel(wxFrame *parent)
    : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(500, 500)) {
  Bind(wxEVT_CHAR, &ControllerPanel::OnKeyDown, this);
  m_paSound = std::make_unique<PAsound>();
  m_paSound.get()->init(0, State::SENDING);
  createLayout();
}

ControllerPanel::~ControllerPanel() {
  Unbind(wxEVT_KEY_DOWN, &ControllerPanel::OnKeyDown, this);
  m_paSound.reset();
}

void ControllerPanel::OnKeyDown(wxKeyEvent &event) {
  int keyCode = event.GetKeyCode();
  switch (keyCode) {
  case 'w':
    m_paSound.get()->play(Operation::FORWARD);
    break;
  case 'a':
    m_paSound.get()->play(Operation::LEFT);
    break;
  case 's':
    m_paSound.get()->play(Operation::BACKWARD);
    break;
  case 'd':
    m_paSound.get()->play(Operation::RIGHT);
    break;
  case 'x':
    m_paSound.get()->play(Operation::STOP);
    break;
  default:
    wxLogMessage("%d", keyCode);
  }

  // Skip the event, so that it can be processed by other handlers
  event.Skip();
}

void ControllerPanel::OnButtonPressed(wxCommandEvent &event) {
  int id = event.GetId();
  if (id >= wxID_HIGHEST + 1 && id <= wxID_HIGHEST + 16) {
    int index = id - wxID_HIGHEST - 1;
    m_paSound.get()->play((DTMF)index);
  } else if (id >= wxID_HIGHEST + 17 && id <= wxID_HIGHEST + 21) {
    switch (id) {
    case wxID_HIGHEST + 17:
      m_paSound.get()->play(Operation::FORWARD);
      break;
    case wxID_HIGHEST + 18:
      m_paSound.get()->play(Operation::LEFT);
      break;
    case wxID_HIGHEST + 19:
      m_paSound.get()->play(Operation::BACKWARD);
      break;
    case wxID_HIGHEST + 20:
      m_paSound.get()->play(Operation::RIGHT);
      break;
    case wxID_HIGHEST + 21:
      m_paSound.get()->play(Operation::STOP);
      break;
    default:
      break;
    }
  }
}

void ControllerPanel::createLayout() {
  wxBoxSizer *mainSizer = new wxBoxSizer(wxVERTICAL);
  wxBoxSizer *buttonSizer = new wxBoxSizer(wxHORIZONTAL);

  for (int i = 0; i < 16; i++) {
    wxButton *button =
        new wxButton(this, wxID_HIGHEST + 1 + i, wxString(indexToDtmf[i]));
    button->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
    buttonSizer->Add(button, 1, wxEXPAND | wxALL, 5);
  }
  wxButton *forwardButton =
      new wxButton(this, wxID_HIGHEST + 17, wxT("Forward(w)"));
  forwardButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *leftButton = new wxButton(this, wxID_HIGHEST + 18, wxT("Left(a)"));
  leftButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *backwardButton =
      new wxButton(this, wxID_HIGHEST + 19, wxT("Backward(s)"));
  backwardButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *rightButton =
      new wxButton(this, wxID_HIGHEST + 20, wxT("Right(d)"));
  rightButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *stopButton = new wxButton(this, wxID_HIGHEST + 21, wxT("Stop(x)"));
  stopButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);

  buttonSizer->Add(forwardButton, 1, wxEXPAND | wxALL, 5);
  buttonSizer->Add(leftButton, 1, wxEXPAND | wxALL, 5);
  buttonSizer->Add(backwardButton, 1, wxEXPAND | wxALL, 5);
  buttonSizer->Add(rightButton, 1, wxEXPAND | wxALL, 5);
  buttonSizer->Add(stopButton, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(buttonSizer, 1, wxEXPAND | wxALL, 5);
  SetSizer(mainSizer);
}
