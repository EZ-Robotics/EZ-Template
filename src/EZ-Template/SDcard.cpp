/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include "EZ-Template/Auton.hpp"
#include "EZ-Template/AutonSelector.hpp"
#include "EZ-Template/Helper.hpp"


namespace ez {
  namespace as {
    AutonSelector autoSelector{};


    void update_auto_sd() {
      // If no SD card, return
      if (!ez::util::IS_SD_CARD) return;

      FILE* usd_file_write = fopen("/usd/auto.txt", "w");
      std::string cp_str = std::to_string(autoSelector.CurrentAutonPage);
      char const *cp_c = cp_str.c_str();
      fputs(cp_c, usd_file_write);
      fclose(usd_file_write);
    }

    void init_sd() {
      // Auton Selector
      FILE *as_usd_file_read = fopen("/usd/auto.txt", "r");
      char buf[5];
      fread(buf, 1, 5, as_usd_file_read);
      autoSelector.CurrentAutonPage = std::stoi(buf);
      fclose(as_usd_file_read);
    }

    void init_auton_selector() {
      // If no SD card, return
      if (!ez::util::IS_SD_CARD)  return;

      if(autoSelector.CurrentAutonPage>autoSelector.AutonCount || autoSelector.CurrentAutonPage<0) {
        autoSelector.CurrentAutonPage=0;
        update_auto_sd();
      }
    }

    void page_up() {

      if (autoSelector.CurrentAutonPage == autoSelector.AutonCount)
        autoSelector.CurrentAutonPage = 0;
      else
        autoSelector.CurrentAutonPage++;
      update_auto_sd();
      autoSelector.PrintSelectedAuto();
    }

    void page_down() {
      if (autoSelector.CurrentAutonPage == 0)
        autoSelector.CurrentAutonPage = autoSelector.AutonCount - 1;
      else
        autoSelector.CurrentAutonPage--;
      update_auto_sd();
      autoSelector.PrintSelectedAuto();
    }
  } // sd namespace
} // ez namespace
