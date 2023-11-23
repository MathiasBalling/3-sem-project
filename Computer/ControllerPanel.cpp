#include "ControllerPanel.h"
#include "consts.h"
#include <string>

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
    m_paSound.get()->play(Operation::FORWARD, {}, m_duration);
    break;
  case 'a':
    m_paSound.get()->play(Operation::LEFT, {}, m_duration);
    break;
  case 's':
    m_paSound.get()->play(Operation::BACKWARD, {}, m_duration);
    break;
  case 'd':
    m_paSound.get()->play(Operation::RIGHT, {}, m_duration);
    break;
  case 'x':
    m_paSound.get()->play(Operation::STOP, {}, m_duration);
    break;
  case 'm':
    m_paSound.get()->play(Operation::MOVEMENT,
                          {m_movement.first, m_movement.second}, m_duration);
    break;
  case 'q':
    m_paSound.get()->stop();
    break;
  default:
    wxLogMessage("'%c' is not a valid key!", keyCode);
  }

  // Skip the event, so that it can be processed by other handlers
  /* event.Skip(); */
}

void ControllerPanel::OnButtonPressed(wxCommandEvent &event) {
  int id = event.GetId();
  if (id >= wxID_HIGHEST + 1 && id <= wxID_HIGHEST + 16) {
    int index = id - wxID_HIGHEST - 1;
    m_paSound.get()->play((DTMF)index, m_duration);
  } else {
    switch (id) {
    case wxID_HIGHEST + 17:
      m_paSound.get()->play(Operation::FORWARD, {}, m_duration);
      break;
    case wxID_HIGHEST + 18:
      m_paSound.get()->play(Operation::LEFT, {}, m_duration);
      break;
    case wxID_HIGHEST + 19:
      m_paSound.get()->play(Operation::BACKWARD, {}, m_duration);
      break;
    case wxID_HIGHEST + 20:
      m_paSound.get()->play(Operation::RIGHT, {}, m_duration);
      break;
    case wxID_HIGHEST + 21:
      m_paSound.get()->play(Operation::STOP, {}, m_duration);
      break;
    case wxID_HIGHEST + 22:
      m_paSound.get()->play(Operation::MOVEMENT,
                            {m_movement.first, m_movement.second}, m_duration);
      break;
    default:
      break;
    }
  }
}

void ControllerPanel::createLayout() {
  // Create main sizer for organizing all elements in the panel
  wxBoxSizer *mainSizer = new wxBoxSizer(wxVERTICAL);

  // Create sizer for Durations
  wxBoxSizer *durationSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *durationText =
      new wxStaticText(this, wxID_ANY, wxT("Duration pr. DTMF tone:"));
  wxTextCtrl *durationCtrl =
      new wxTextCtrl(this, wxID_ANY, std::to_string(m_duration));
  durationCtrl->SetValidator(wxTextValidator(wxFILTER_DIGITS));
  durationCtrl->SetMinSize(wxSize(70, -1));
  durationCtrl->Bind(wxEVT_TEXT, [&](wxCommandEvent &event) {
    m_duration = wxAtoi(event.GetString());
  });
  wxStaticText *durationUnitText = new wxStaticText(this, wxID_ANY, wxT("ms"));
  durationSizer->Add(durationText, 0, wxALIGN_CENTRE_VERTICAL | wxALL, 5);
  durationSizer->Add(durationCtrl, 0, wxALIGN_CENTRE_VERTICAL);
  durationSizer->Add(durationUnitText, 0, wxALIGN_CENTRE_VERTICAL | wxLEFT, 5);

  // Create sizer for Movement
  wxStaticText *movementText =
      new wxStaticText(this, wxID_ANY, wxT("Movement:"));
  wxBoxSizer *movementSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *movementLinearText =
      new wxStaticText(this, wxID_ANY, wxT("Linear = "));
  wxTextCtrl *movementLinearCtrl = new wxTextCtrl(this, wxID_ANY, wxT("0.0"));
  movementLinearCtrl->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
  movementLinearCtrl->SetMinSize(wxSize(70, -1));
  movementLinearCtrl->Bind(wxEVT_TEXT, [&](wxCommandEvent &event) {
    m_movement.first = wxAtof(event.GetString());
  });
  wxStaticText *movementAngularText =
      new wxStaticText(this, wxID_ANY, wxT("Angular = "));
  wxTextCtrl *movementAngularCtrl = new wxTextCtrl(this, wxID_ANY, wxT("0.0"));
  movementAngularCtrl->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
  movementAngularCtrl->SetMinSize(wxSize(70, -1));
  movementAngularCtrl->Bind(wxEVT_TEXT, [&](wxCommandEvent &event) {
    m_movement.second = wxAtof(event.GetString());
  });
  movementSizer->Add(movementText, 0, wxALIGN_CENTRE_VERTICAL | wxALL, 5);
  movementSizer->Add(movementLinearText, 0, wxALIGN_CENTRE_VERTICAL | wxLEFT,
                     5);
  movementSizer->Add(movementLinearCtrl, 0, wxALIGN_CENTRE_VERTICAL);
  movementSizer->Add(movementAngularText, 0, wxALIGN_CENTRE_VERTICAL | wxLEFT,
                     10);
  movementSizer->Add(movementAngularCtrl, 0, wxALIGN_CENTRE_VERTICAL);

  // Create DTMF buttons and add them to sizer
  wxGridSizer *dtmfButtonSizer = new wxGridSizer(4, 4, 0, 0);
  for (int i = 0; i < 16; i++) {
    wxButton *button =
        new wxButton(this, wxID_HIGHEST + 1 + i, wxString(indexToDtmf[i]));
    button->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
    dtmfButtonSizer->Add(button, 1, wxEXPAND | wxALL, 5);
  }

  // Create operation buttons
  wxButton *forwardButton =
      new wxButton(this, wxID_HIGHEST + 17, wxT("Forward (w)"));
  forwardButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *leftButton = new wxButton(this, wxID_HIGHEST + 18, wxT("Left (a)"));
  leftButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *backwardButton =
      new wxButton(this, wxID_HIGHEST + 19, wxT("Backward (s)"));
  backwardButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *rightButton =
      new wxButton(this, wxID_HIGHEST + 20, wxT("Right (d)"));
  rightButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  // Create sizer for operation buttons and add them to sizer
  wxBoxSizer *movementButtonsSizer = new wxBoxSizer(wxHORIZONTAL);
  movementButtonsSizer->Add(forwardButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(leftButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(backwardButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(rightButton, 1, wxEXPAND | wxALL, 5);

  wxButton *stopButton = new wxButton(this, wxID_HIGHEST + 21, wxT("Stop (x)"));
  stopButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *movementButton =
      new wxButton(this, wxID_HIGHEST + 22, wxT("Movement (m)"));
  movementButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxBoxSizer *utilButtonSizer = new wxBoxSizer(wxHORIZONTAL);
  utilButtonSizer->Add(movementButton, 1, wxEXPAND | wxALL, 5);
  utilButtonSizer->Add(stopButton, 1, wxEXPAND | wxALL, 5);

  // Add sizers to main sizer
  mainSizer->Add(durationSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(movementSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(movementButtonsSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(utilButtonSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(dtmfButtonSizer, 1, wxEXPAND | wxALL, 5);
  SetSizerAndFit(mainSizer);
}
