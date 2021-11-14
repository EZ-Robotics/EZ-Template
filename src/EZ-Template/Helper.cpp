/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "main.h"



namespace ez {
 int mode = DISABLE;

void print_ez_template() {
  std::cout << R"(


    _____ ______   _____                    _       _
   |  ___|___  /  |_   _|                  | |     | |
   | |__    / /_____| | ___ _ __ ___  _ __ | | __ _| |_ ___
   |  __|  / /______| |/ _ \ '_ ` _ \| '_ \| |/ _` | __/ _ \
   | |___./ /___    | |  __/ | | | | | |_) | | (_| | ||  __/
   \____/\_____/    \_/\___|_| |_| |_| .__/|_|\__,_|\__\___|
                                     | |
                                     |_|
)" << '\n';

  printf("Version: 2.0.0\n");
}
std::string get_last_word(std::string& text)
{
  std::string word = "";
  for(int i = text.length() - 1; i > 0; i--)
  {
    if(text[i] != ' ')
    {
      word+=text[i];
    }
    else
    {
      return word;
    }
    text.erase(i, 1);
  }
  return word;
}
std::string get_rest_of_the_word(std::string text, int& position)
{
  std::string word = "";
  for(int i = position; i < text.length(); i++)
  {
    if(text[i] != ' ')
    {
      word+=text[i];
    }
    else
    {
      position = i;
      return word;
    }
  }
  position = text.length();
  return word;
}
void print_to_screen(std::string text, int line)
{
  int CurrAutoLine = line;
  std::vector<string> texts = {};
  std::string temp = "";

  for(int i = 0; i < text.length(); i++)
  {
    if(temp.length() == 20)//change 20 to correct number
    {
      auto last_word = get_last_word(temp);
      last_word += get_rest_of_the_word(temp, i);
      texts.push_back(temp);
      temp = last_word;
    }
    else if(text[i] == '\n')
    {
      if(temp != "")
      {
        texts.push_back(temp);
      }
      texts.emplace_back("");
      temp = "";
    }
    else if(i == text.length() - 1)
    {
      temp += text[i];
      texts.push_back(temp);
      temp = "";
    }
    else
    {
      temp += text[i];
    }
  }
  for(auto i : texts)
  {
    if(CurrAutoLine>7)
    {
      pros::lcd::clear();
      pros::lcd::set_text(0, "Out of Bounds. Print Line is too far down");
      return;
    }
    pros::lcd::clear_line(CurrAutoLine);
    pros::lcd::set_text(CurrAutoLine, i);
    CurrAutoLine++;
  }
}
namespace util {
  bool AUTON_RAN = true;

bool is_reversed(double input) {
  if (input < 0)
    return true;
  return false;
}

int sgn (double input) {
  if (input > 0)
    return 1;
  else if (input < 0)
    return -1;
  return 0;
}

double clip_num(double input, double max, double min) {
  if (input > max)
    return max;
  else if (input < min)
    return min;
  return input;
}

} // util namespace
} // ez namespace
