

int melody[] = {
  NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
  NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_A4,
  NOTE_B4, NOTE_C5, NOTE_C5, 0,
  NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
  NOTE_A4
};

int noteDurations[] = {
  250, 250, 500, 250, 125,
  250, 250, 500, 250, 125,
  250, 250, 500, 250, 125,
  250, 250, 500, 250, 250,
  250, 250, 500, 250, 125,
  250, 250, 500, 250, 125,
  250
};

void playPiratesTheme() {
  int notes = sizeof(melody) / sizeof(melody[0]);
  for (int i = 0; i < notes; i++) {
    int duration = noteDurations[i];
    int note = melody[i];
    if (note == 0) {
      noTone(BUZZER);
    } else {
      tone(BUZZER, note, duration);
    }
    delay(duration * 1.3);
  }
  noTone(BUZZER);
}

void warningBeep() {
  for (int i = 0; i < 3; i++) {
    tone(BUZZER, NOTE_A5, 200);
    delay(3000);
  }
  noTone(BUZZER);
}