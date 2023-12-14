#include "ControllerPanel.h"
#include "consts.h"
#include "protocol.h"
#include <string>
#include <thread>

ControllerPanel::ControllerPanel(wxFrame *parent)
    : wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(500, 500)) {

  m_paSound = std::make_unique<PAsound>();
  bool initialized = m_paSound->init(0, State::SENDING);
  if (!initialized) {
    wxLogMessage("PortAudio failed to initialize!");
  } else {
    wxLogMessage("PortAudio initialized!");
  }

  createLayout();

  Bind(wxEVT_CHAR, &ControllerPanel::OnKeyDown, this);
  m_timer.Bind(wxEVT_TIMER, &ControllerPanel::OnTimer, this);
}

ControllerPanel::~ControllerPanel() {
  Unbind(wxEVT_KEY_DOWN, &ControllerPanel::OnKeyDown, this);

  // Disable input to stop thread
  m_paSound->setState(State::SENDING);
  if (m_thread.joinable())
    m_thread.join();
}

void ControllerPanel::handleAudioInput() const {
  wxLogMessage("Thread started!");
  while (m_paSound->getState() != State::SENDING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    if (m_paSound->getState() == State::PROCESSING) {
      auto data = m_paSound->processInput();
      auto op = getOperation(data);
      wxLogMessage("Operation: %s", indexToOperation[(int)op]);
      if (op == Operation::MOVEMENT) {
        wxLogMessage("Linear: %f, Angular: %f", data[0], data[1]);
      } else if (op > Operation::MOVEMENT) {
        wxLogMessage("%f, %f", data[0], data[1]);
      }
    }
  }
  wxLogMessage("Thread stopped!");
}

void ControllerPanel::OnTimer(wxTimerEvent &event) {
  (void)event; // To prevent unused variable warning
  m_isReadyToPlay = true;
}

void ControllerPanel::OnKeyDown(wxKeyEvent &event) {
  int keyCode = event.GetKeyCode();
  if (keyCode == 'q')
    m_paSound->stop();

  if (!m_isReadyToPlay) {
    return;
  }
  switch (keyCode) {
  case 'w':
    m_paSound->play(Operation::FORWARD, {}, m_duration);
    break;
  case 'a':
    m_paSound->play(Operation::LEFT, {}, m_duration);
    break;
  case 's':
    m_paSound->play(Operation::BACKWARD, {}, m_duration);
    break;
  case 'd':
    m_paSound->play(Operation::RIGHT, {}, m_duration);
    break;
  case 'x':
    m_paSound->play(Operation::STOP, {}, m_duration);
    break;
  case 'm':
    m_paSound->play(Operation::MOVEMENT, {m_movement.first, m_movement.second},
                    m_duration);
    break;
  case 'q':
    m_paSound->stop();
    return;
  default:
    wxLogMessage("'%c' is not a valid key!", keyCode);
    return;
  }

  // Prevent the user from spamming the keyboard
  m_isReadyToPlay = false;
  m_timer.StartOnce(m_duration * 5); // 5 times the duration of a DTMF tone

  // Skip the event, so that it can be processed by other handlers
  /* event.Skip(); */
}

void ControllerPanel::OnButtonPressed(wxCommandEvent &event) {
  if (!m_isReadyToPlay) {
    return;
  }
  // Get the id of the button that was pressed and play the corresponding DTMF
  // or operation
  int id = event.GetId();
  if (id >= wxID_HIGHEST + 1 && id <= wxID_HIGHEST + 16) {
    int index = id - wxID_HIGHEST - 1;
    m_paSound->play((DTMF)index, m_duration);
  } else {
    switch (id) {
    case wxID_HIGHEST + 17:
      m_paSound->play(Operation::FORWARD, {}, m_duration);
      break;
    case wxID_HIGHEST + 18:
      m_paSound->play(Operation::LEFT, {}, m_duration);
      break;
    case wxID_HIGHEST + 19:
      m_paSound->play(Operation::BACKWARD, {}, m_duration);
      break;
    case wxID_HIGHEST + 20:
      m_paSound->play(Operation::RIGHT, {}, m_duration);
      break;
    case wxID_HIGHEST + 21:
      m_paSound->play(Operation::STOP, {}, m_duration);
      break;
    case wxID_HIGHEST + 22:
      m_paSound->play(Operation::MOVEMENT,
                      {m_movement.first, m_movement.second}, m_duration);
      break;
    case wxID_HIGHEST + 23:
      m_paSound->stop();
      break;
    case wxID_HIGHEST + 24:
      m_paSound->play(Operation::UPDATE_MAG_THRESHOLD,
                      {m_paSound->getMinMagnitude()}, m_duration);
      break;
    default:
      return;
    }
  }

  // Prevent the user from spamming the buttons
  m_isReadyToPlay = false;
  m_timer.StartOnce(m_duration * 5);
}

void ControllerPanel::createLayout() {
  // Create main sizer for organizing all elements in the panel
  wxBoxSizer *mainSizer = new wxBoxSizer(wxVERTICAL);

  // For magnification threshold
  wxBoxSizer *magnificationThresholdSizer = new wxBoxSizer(wxHORIZONTAL);
  wxStaticText *magnificationThresholdText =
      new wxStaticText(this, wxID_ANY, wxT("Magnification threshold:"));
  wxTextCtrl *magnificationThresholdCtrl = new wxTextCtrl(
      this, wxID_ANY, std::to_string(m_paSound->getMinMagnitude()));
  magnificationThresholdCtrl->SetValidator(wxTextValidator(wxFILTER_NUMERIC));
  magnificationThresholdCtrl->SetMinSize(wxSize(70, -1));
  magnificationThresholdCtrl->Bind(wxEVT_TEXT, [&](wxCommandEvent &event) {
    m_paSound->setMinMagnitude(wxAtof(event.GetString()));
  });
  wxStaticText *magnificationThresholdUnitText =
      new wxStaticText(this, wxID_ANY, wxT("1/fs"));
  magnificationThresholdSizer->Add(magnificationThresholdText, 0,
                                   wxALIGN_CENTRE_VERTICAL | wxALL, 5);
  magnificationThresholdSizer->Add(magnificationThresholdCtrl, 0,
                                   wxALIGN_CENTRE_VERTICAL);
  magnificationThresholdSizer->Add(magnificationThresholdUnitText, 0,
                                   wxALIGN_CENTRE_VERTICAL | wxLEFT, 5);

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
  wxCheckBox *isListeningCheckBox =
      new wxCheckBox(this, wxID_ANY, wxT("Listen to sound"));

  bool isListening = m_paSound->getState() == State::SENDING ? false : true;
  isListeningCheckBox->SetValue(isListening);
  isListeningCheckBox->Bind(wxEVT_CHECKBOX, [&](wxCommandEvent &event) {
    if (event.IsChecked()) {
      m_paSound->setState(State::LISTENING);
      m_thread = std::thread(&ControllerPanel::handleAudioInput, this);
    } else {
      m_paSound->setState(State::SENDING);
      m_paSound->stop();
      m_thread.join();
    }
  });
  durationSizer->Add(durationText, 0, wxALIGN_CENTRE_VERTICAL | wxALL, 5);
  durationSizer->Add(durationCtrl, 0, wxALIGN_CENTRE_VERTICAL);
  durationSizer->Add(durationUnitText, 0, wxALIGN_CENTRE_VERTICAL | wxLEFT, 5);
  durationSizer->Add(isListeningCheckBox, 0,
                     wxALIGN_RIGHT | wxALIGN_CENTER_VERTICAL | wxLEFT, 10);

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
  wxButton *stopButton = new wxButton(this, wxID_HIGHEST + 21, wxT("Stop (x)"));
  stopButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  // Create sizer for operation buttons and add them to sizer
  wxBoxSizer *movementButtonsSizer = new wxBoxSizer(wxHORIZONTAL);
  movementButtonsSizer->Add(forwardButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(backwardButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(leftButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(rightButton, 1, wxEXPAND | wxALL, 5);
  movementButtonsSizer->Add(stopButton, 1, wxEXPAND | wxALL, 5);

  wxButton *movementButton =
      new wxButton(this, wxID_HIGHEST + 22, wxT("Movement (m)"));
  movementButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);
  wxButton *stopSoundButton =
      new wxButton(this, wxID_HIGHEST + 23, wxT("Stop sound (q)"));
  stopSoundButton->Bind(wxEVT_BUTTON, &ControllerPanel::OnButtonPressed, this);

  wxButton *thresholdButton =
      new wxButton(this, wxID_HIGHEST + 24, wxT("Set threshold"));
  thresholdButton->Bind(wxEVT_BUTTON, [&](wxCommandEvent &event) {
    (void)event; // To prevent unused variable warning
    m_paSound->play(Operation::UPDATE_MAG_THRESHOLD,
                    {m_paSound->getMinMagnitude()}, m_duration);
  });
  wxBoxSizer *utilButtonSizer = new wxBoxSizer(wxHORIZONTAL);
  utilButtonSizer->Add(movementButton, 1, wxEXPAND | wxALL, 5);
  utilButtonSizer->Add(stopSoundButton, 1, wxEXPAND | wxALL, 5);
  utilButtonSizer->Add(thresholdButton, 1, wxEXPAND | wxALL, 5);

  // Add sizers to main sizer
  mainSizer->Add(magnificationThresholdSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(durationSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(movementSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(movementButtonsSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(utilButtonSizer, 1, wxEXPAND | wxALL, 5);
  mainSizer->Add(dtmfButtonSizer, 1, wxEXPAND | wxALL, 5);
  SetSizerAndFit(mainSizer);
}
