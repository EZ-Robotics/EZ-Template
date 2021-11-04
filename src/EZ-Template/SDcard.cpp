/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/SDcard.hpp"
#include  "EZ-Template/AutonSelector.hpp"

namespace ez
{
  namespace sd
  {
    //AutonSelector autonSelector{};
    const bool IS_SD_CARD = pros::usd::is_installed();

    AutonSelector temp;

    void init_auton_selector(AutonSelector selector) {
      // If no SD card, return
      if (!IS_SD_CARD)  return;

      // Auton Selector
      FILE* usd_file_read = fopen("/usd/auto.txt", "r");
      char buf[5];
      fread(buf, 1, 5, usd_file_read);
      temp.CurrentAutonPage = std::stoi(buf);
      fclose(usd_file_read);

      if(temp.CurrentAutonPage>temp.AutonCount-1 || temp.CurrentAutonPage<0)
      {
        temp.CurrentAutonPage=0;
        update_auto_sd();
      }
    }

    void update_auto_sd() {
      // If no SD card, return
      if (!IS_SD_CARD) return;

      FILE* usd_file_write = fopen("/usd/auto.txt", "w");
      std::string cp_str = std::to_string(temp.CurrentAutonPage);
      char const *cp_c = cp_str.c_str();
      fputs(cp_c, usd_file_write);
      fclose(usd_file_write);
    }

    void page_up()
    {

      if (temp.CurrentAutonPage == temp.AutonCount - 1)
        temp.CurrentAutonPage = 0;
      else
        temp.CurrentAutonPage++;
      update_auto_sd();
      temp.PrintSelectedAuto();
    }

    void page_down()
    {
      if (temp.CurrentAutonPage == 0)
        temp.CurrentAutonPage = temp.AutonCount - 1;
      else
        temp.CurrentAutonPage--;
      update_auto_sd();
      temp.PrintSelectedAuto();
    }
  } // sd namespace
} // ez namespace
