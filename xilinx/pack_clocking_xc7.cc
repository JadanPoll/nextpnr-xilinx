/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019  David Shah <david@symbioticeda.com>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <algorithm>
#include <boost/optional.hpp>
#include <iterator>
#include <queue>
#include <unordered_set>
#include "cells.h"
#include "chain_utils.h"
#include "design_utils.h"
#include "log.h"
#include "nextpnr.h"
#include "pack.h"
#include "pins.h"

NEXTPNR_NAMESPACE_BEGIN

void XC7Packer::prepare_clocking()
{
    log_info("Preparing clocking...\n");
    dict<IdString, IdString> upgrade;
    upgrade[id_MMCME2_BASE] = id_MMCME2_ADV;
    upgrade[id_PLLE2_BASE] = id_PLLE2_ADV;

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (upgrade.count(ci->type)) {
            IdString new_type = upgrade.at(ci->type);
            ci->type = new_type;
        } else if (ci->type == id_BUFG) {
            ci->type = id_BUFGCTRL;
            ci->renamePort(id_I, id_I0);
            tie_port(ci, "CE0", true, true);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
        } else if (ci->type == id_BUFGCE) {
            ci->type = id_BUFGCTRL;
            ci->renamePort(id_I, id_I0);
            ci->renamePort(id_CE, id_CE0);
            tie_port(ci, "S0", true, true);
            tie_port(ci, "S1", false, true);
            tie_port(ci, "IGNORE0", true, true);
        } 
        // Nathan: Yosys outputs BUFR as type string "BUFR" (not in constids).
        // Map to id_BUFR_BUFR for chipdb BEL matching.
        // BUFR has no config bits — purely a routing resource.
        // fasm.cc already handles IN_USE and BUFR_DIVIDE.BYPASS via pp_config routing.
        else if (ci->type == ctx->id("BUFR")) {
            ci->type = id_BUFR_BUFR;
        } 
        
        // Nathan: Yosys outputs BUFIO as type string "BUFIO" (not in constids).
        // Map to id_BUFIO_BUFIO for chipdb BEL matching.
        // BUFIO is a purely routing resource — no config bits in prjxray DB.
        // Used for source-synchronous interfaces requiring regional clock routing.
        else if (ci->type == ctx->id("BUFIO")) {
            ci->type = id_BUFIO_BUFIO;
        } else if (ci->type == id_BUFHCE) {
            // Nathan: BUFHCE — horizontal clock buffer with clock enable.
            // BEL type BUFHCE_BUFHCE in CLK_HROW tile.
            // CE_TYPE (SYNC/ASYNC) and INIT_OUT handled in fasm.cc.
            // ZINV_CE and IN_USE emitted via pp_config routing (fasm.cc line 174).
            // IS_CE_INVERTED handled via invertible_pins[id_BUFHCE] in pins.cc.
            ci->type = ctx->id("BUFHCE_BUFHCE");
        }
        else if (ci->type == ctx->id("XADC")) {
               
            // Nathan: XADC placement fix.
            // Yosys outputs type string "XADC" — must convert to id_XADC (constid)
            // before calling preplace_unique, which matches cell->type against chipdb BEL types.
            // XADC is a single unique hard macro per device — preplace_unique finds
            // the only XADC BEL (XADC_X0Y0/XADC) and assigns it.
            // Confirmed from Vivado: LOC=XADC_X0Y0, BEL=XADC.XADC, tile=MONITOR_BOT_X46Y79.
            // pins.cc invertible pins (CONVSTCLK, DCLK) already defined for id_XADC.
            // INIT param FASM emission pending segbits_xadc.db generation via 033-mon-xadc fuzzer.
            // fasm.cc routing pips (CONVSTCLKINV, DCLKINV) handled via site_pips automatically.
            ci->type = id_XADC;
                        
            // Nathan: VP and VN are dedicated analog inputs with no fabric routing path.
            // Disconnect them so the router doesn't try to route GND/constants to them.
            // Confirmed: nextpnr fails with "Failed to route arc PSEUDO_GND_NET to SITEWIRE/XADC_X0Y0/VP"
            // when VP/VN are connected to constants. Vivado handles VP/VN outside the routing fabric.
            for (auto port : {ctx->id("VP"), ctx->id("VN")}) {
                NetInfo *net = ci->getPort(port);
                if (net != nullptr)
                    ci->disconnectPort(port);
            }
            
            
            preplace_unique(ci);
        }



        if (ci->attrs.count(id_BEL))
            used_bels.insert(ctx->getBelByNameStr(ci->attrs.at(id_BEL).as_string()));
    }
}

void XC7Packer::pack_plls()
{
    log_info("Packing PLLs...\n");

    auto set_default = [](CellInfo *ci, IdString param, const Property &value) {
        if (!ci->params.count(param))
            ci->params[param] = value;
    };

    dict<IdString, XFormRule> pll_rules;
    pll_rules[id_MMCME2_ADV].new_type = id_MMCME2_ADV_MMCME2_ADV;
    pll_rules[id_PLLE2_ADV].new_type = id_PLLE2_ADV_PLLE2_ADV;
    generic_xform(pll_rules);
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        // Preplace PLLs to make use of dedicated/short routing paths
        if (ci->type.in(id_MMCM_MMCM_TOP, id_PLL_PLL_TOP, id_MMCME2_ADV_MMCME2_ADV, id_PLLE2_ADV_PLLE2_ADV))
            try_preplace(ci, id_CLKIN1);
        if (ci->type == id_MMCM_MMCM_TOP) {
            // Fixup parameters
            for (int i = 1; i <= 2; i++)
                set_default(ci, ctx->id("CLKIN" + std::to_string(i) + "_PERIOD"), Property("0.0"));
            for (int i = 0; i <= 6; i++) {
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_CASCADE"), Property("FALSE"));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_DIVIDE"), Property(1));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_DUTY_CYCLE"), Property("0.5"));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_PHASE"), Property(0));
                set_default(ci, ctx->id("CLKOUT" + std::to_string(i) + "_USE_FINE_PS"), Property("FALSE"));
            }
            set_default(ci, id_COMPENSATION, Property("INTERNAL"));

            // Fixup routing
            if (str_or_default(ci->params, id_COMPENSATION, "INTERNAL") == "INTERNAL") {
                ci->disconnectPort(id_CLKFBIN);
                ci->connectPort(id_CLKFBIN, ctx->nets[ctx->id("$PACKER_VCC_NET")].get());
            }
        }
    }
}


void XC7Packer::pack_gbs()
{
    log_info("Entering XC7Packer::pack_gbs()...\n");
    log_info("Packing global buffers...\n");
    
    log_info("Initializing gb_rules dictionary.\n");
    dict<IdString, XFormRule> gb_rules;
    
    log_info("Setting rule for id_BUFGCTRL to map to id_BUFGCTRL.\n");
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;
    
    // Note: This line was duplicated in your original code.
    log_info("Setting redundant rule for id_BUFGCTRL to map to id_BUFGCTRL.\n");
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;

    log_info("Executing generic_xform(gb_rules)...\n");
    generic_xform(gb_rules);
    log_info("Completed generic_xform(gb_rules).\n");

    log_info("Starting prerequisite setup loop (looking for PS7_PS7)...\n");
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        log_info("  Evaluating cell '%s' for prerequisites. Type: '%s'\n", ctx->nameOf(ci), ci->type.c_str(ctx));
        
        if (ci->type == id_PS7_PS7) {
            log_info("    Match found: Cell '%s' is id_PS7_PS7. Calling preplace_unique().\n", ctx->nameOf(ci));
            preplace_unique(ci);
        } else {
            log_info("    Cell '%s' is not id_PS7_PS7. Skipping.\n", ctx->nameOf(ci));
        }
    }
    log_info("Completed prerequisite setup loop.\n");

    log_info("Starting main global buffer preplacement loop...\n");
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        log_info("  Evaluating cell '%s' (Type: '%s') for preplacement.\n", ctx->nameOf(ci), ci->type.c_str(ctx));

        if (ci->type == id_BUFGCTRL) {
            log_info("    Match: '%s' is id_BUFGCTRL. Calling try_preplace() on port id_I0.\n", ctx->nameOf(ci));
            try_preplace(ci, id_I0);
        }
        
        if (ci->type == id_BUFG_BUFG) {
            log_info("    Match: '%s' is id_BUFG_BUFG. Calling try_preplace() on port id_I.\n", ctx->nameOf(ci));
            try_preplace(ci, id_I);
        }

        // Nathan: Preplace BUFR near its clock source via dedicated routing.
        log_info("    Checking if cell '%s' is BUFR or BUFIO...\n", ctx->nameOf(ci));
        if (ci->type == id_BUFR_BUFR || ci->type == id_BUFIO_BUFIO) {
            log_info("    Match: '%s' is BUFR/BUFIO type.\n", ctx->nameOf(ci));
            log_info("    BUFR/BUFIO '%s' type='%s' has_bel=%d bel='%s'\n",
                ctx->nameOf(ci), ci->type.c_str(ctx),
                ci->attrs.count(id_BEL) ? 1 : 0,
                ci->attrs.count(id_BEL) ? ci->attrs.at(id_BEL).as_string().c_str() : "none");

            log_info("    Evaluating if cell '%s' lacks a BEL attribute...\n", ctx->nameOf(ci));
            if (!ci->attrs.count(id_BEL)) {
                log_info("      Cell lacks BEL. Fetching port id_I...\n");
                NetInfo *n = ci->getPort(id_I);
                
                log_info("      Evaluating port id_I validity and driver cell...\n");
                if (n && n->driver.cell) {
                    CellInfo *drv = n->driver.cell;
                    log_info("      Port valid. BUFR '%s': driver cell '%s' type '%s'\n",
                        ctx->nameOf(ci), ctx->nameOf(drv), drv->type.c_str(ctx));

                    log_info("      Initializing pad_cell to nullptr.\n");
                    CellInfo *pad_cell = nullptr;
                    
                    log_info("      Fetching port id_PAD from driver cell...\n");
                    NetInfo *pad_net = drv->getPort(id_PAD);
                    
                    log_info("      Evaluating pad_net validity and attributes...\n");
                    if (pad_net && pad_net->driver.cell && pad_net->driver.cell->attrs.count(id_BEL)) {
                        log_info("        pad_net driver cell has BEL. Assigning to pad_cell.\n");
                        pad_cell = pad_net->driver.cell;
                    } else {
                        log_info("        pad_net driver cell invalid or lacks BEL.\n");
                    }
                    
                    log_info("      Evaluating fallback: checking if driver cell itself has BEL...\n");
                    if (!pad_cell && drv->attrs.count(id_BEL)) {
                        log_info("        Fallback matched. Assigning driver cell to pad_cell.\n");
                        pad_cell = drv;
                    } else {
                        log_info("        Fallback failed or pad_cell already populated.\n");
                    }

                    if (!pad_cell) {
                        log_info("    BUFR '%s': no pad_cell found, falling back to preplace_unique.\n",
                            ctx->nameOf(ci));
                    } else {
                        log_info("      pad_cell found. Extracting BEL string...\n");
                        std::string pad_site = pad_cell->attrs.at(id_BEL).as_string();
                        log_info("    BUFR '%s': pad_cell '%s' bel '%s'\n",
                            ctx->nameOf(ci), ctx->nameOf(pad_cell), pad_site.c_str());

                        // Diagnostic logging for tile coordinates
                        log_info("      Translating pad_site '%s' to BelId...\n", pad_site.c_str());
                        BelId pad_bel_tmp = ctx->getBelByNameStr(pad_site);
                        
                        log_info("      Calculating pad_tile_y...\n");
                        int pad_tile_y = pad_bel_tmp.tile / ctx->chip_info->width;
                        log_info("    BUFR '%s': pad_bel_tile index=%d tile_y=%d\n",
                            ctx->nameOf(ci), pad_bel_tmp.tile, pad_tile_y);

                        log_info("      Searching for 'Y' in pad_site...\n");
                        auto y_pos = pad_site.find('Y');
                        
                        if (y_pos != std::string::npos) {
                            log_info("      'Y' found at position %zu. Extracting site_y...\n", y_pos);
                            int site_y = std::stoi(pad_site.substr(y_pos + 1,
                                pad_site.find('/', y_pos) - y_pos - 1));
                            
                            log_info("      Formatting site_y_str...\n");

                            // Nathan: Fixed BUFR/BUFIO placement.
                            // I-wire format is SITEWIRE/BUFR_X0Y{N}/I — does NOT contain the IOB site Y.
                            // The site_y_str filter was wrong and never matched.
                            //
                            // Correct approach: match BUFR candidates by tile_y == ibuf tile_y.
                            // From Vivado: HCLK_IOI3_X1Y26 contains BUFR_X0Y0/Y1/Y2/Y3.
                            // nextpnr tile_y for those BUFRs = 26 = IOB site_y. Tile_y computed as bel.tile/width.
                            // Among BELs at tile_y==site_y, select by local Y mod 4 using corrected law:
                            //   clk_idx driven by IOB = (site_y % 4 + 1) % 4
                            //   target local_Y = (clk_idx + 2) % 4 = (site_y % 4 + 3) % 4
                            // Verified: site_y=26 -> target_mod4=1 -> nextpnr Y9 = Vivado BUFR_X0Y1 (CLK3). ✓

                            log_info("    Calculating target_mod4 logic: site_y=%d\n", site_y);
                            
                            // 1. Map IOB site_y to HCLK tile_y FIRST
                            int hclk_tile_y;
                            if (site_y < 52)       hclk_tile_y = 26;
                            else if (site_y < 104) hclk_tile_y = 78;
                            else                   hclk_tile_y = 130;

                            // 2. NOW calculate target_mod4 using the newly defined hclk_tile_y
                            int base_iob = (hclk_tile_y == 26) ? 21 : (hclk_tile_y - 9);
                            int rank = ((site_y - base_iob) / 2) % 4;
                            int target_mod4 = (hclk_tile_y == 26) ? (rank + 2) % 4 : rank;

                            log_info("    BUFR '%s': site_y=%d target_mod4=%d searching for BUFR at tile_y==%d\n",
                                ctx->nameOf(ci), site_y, target_mod4, site_y);
                            log_info("    Starting BEL search loop...\n");

                            for (auto bel : ctx->getBels()) {
                                // Silently skip non-BUFR/BUFIO BELs to prevent console flooding
                                if (ctx->getBelType(bel) != ci->type) {
                                    continue;
                                }

                                std::string bel_name_raw = std::string(ctx->nameOfBel(bel));
                                log_info("      Evaluating candidate BEL '%s' (matches type '%s').\n", bel_name_raw.c_str(), ci->type.c_str(ctx));

                                if (used_bels.count(bel)) {
                                    log_info("        Candidate '%s' is already in used_bels. Skipping.\n", bel_name_raw.c_str());
                                    continue;
                                }

                                log_info("        Calculating candidate_tile_y (bel.tile=%d / chip_width=%d)...\n", bel.tile, ctx->chip_info->width);
                                int candidate_tile_y = bel.tile / ctx->chip_info->width;

                                log_info("        Comparing candidate_tile_y=%d with site_y=%d...\n", candidate_tile_y, site_y);


                                // Map IOB site_y to HCLK tile_y
log_info("    Translating IOB site_y=%d to HCLK clock domain...\n", site_y);
int hclk_tile_y;
if (site_y < 52) {
    hclk_tile_y = 26;
    log_info("      Range 0-51: Target HCLK Y = 26\n");
} else if (site_y < 104) {
    hclk_tile_y = 78;
    log_info("      Range 52-103: Target HCLK Y = 78\n");
} else {
    hclk_tile_y = 130;
    log_info("      Range 104-155: Target HCLK Y = 130\n");
}

// Inside your BEL loop:
if (ctx->getBelType(bel) == ci->type) {
    int candidate_tile_y = bel.tile / ctx->chip_info->width;


    // Use bel_y range for HCLK group — chipdb tile_y is unreliable for BUFIO/BUFR.
// Confirmed Vivado: bel_y 0-3→Y26, 4-7→Y78, 8-11→Y130 (both X0 and X1 banks).
{
    std::string bn_tmp = std::string(ctx->nameOfBel(bel));
    auto by_tmp = bn_tmp.rfind('Y');
    if (by_tmp == std::string::npos) continue;
    int by_tmp_val = std::stoi(bn_tmp.substr(by_tmp + 1));
    int bel_hclk_y = (by_tmp_val < 4) ? 26 : (by_tmp_val < 8) ? 78 : 130;
    if (bel_hclk_y != hclk_tile_y) continue;
}


    log_info("      Tile Y Match! Found candidate '%s' in HCLK tile %d. Checking local mod4...\n", 
             ctx->nameOfBel(bel), hclk_tile_y);
}



                                log_info("        Tile Y match! Extracting local Y coordinate from bel_name...\n");
                                std::string bel_name = std::string(ctx->nameOfBel(bel));
                                auto by_pos = bel_name.rfind('Y');
                                
                                if (by_pos == std::string::npos) {
                                    log_info("        Could not find 'Y' in candidate bel_name '%s'. Skipping.\n", bel_name.c_str());
                                    continue;
                                }

                                log_info("        'Y' found at pos %zu. Parsing integer suffix...\n", by_pos);
                                int bel_y = std::stoi(bel_name.substr(by_pos + 1));
                                
                                int bel_y_mod4 = bel_y % 4;
                                log_info("        Parsed bel_y=%d. Checking local group index: (bel_y %% 4) = %d. Expecting target_mod4=%d...\n", bel_y, bel_y_mod4, target_mod4);
                                
                                if (bel_y_mod4 != target_mod4) {
                                    log_info("        Local group index mismatch. Skipping candidate.\n");
                                    continue;
                                }

                                log_info("        >>> ALL CONDITIONS MET <<< Marking BEL '%s' as used.\n", bel_name.c_str());
                                used_bels.insert(bel);
                                ci->attrs[id_BEL] = bel_name;
                                
                                log_info("    Constrained %s '%s' to bel '%s' (tile_y=%d bel_y=%d target_mod4=%d)\n",
                                    ci->type.c_str(ctx), ctx->nameOf(ci),
                                    bel_name.c_str(), candidate_tile_y, bel_y, target_mod4);
                                
                                break;
                            }
                        } else {
                            log_info("      'Y' not found in pad_site '%s'. Skipping tile search.\n", pad_site.c_str());
                        }
                    }
                } else {
                    log_info("      Port id_I or its driver cell is invalid.\n");
                }
            } else {
                 log_info("      Cell '%s' already has a BEL attribute. Skipping dedicated placement routing.\n", ctx->nameOf(ci));
            }
            
            log_info("    Final check: does cell '%s' have a BEL attribute after BUFR/BUFIO logic?\n", ctx->nameOf(ci));
            if (!ci->attrs.count(id_BEL)) {
                log_info("      Cell '%s' still lacks BEL. Calling preplace_unique().\n", ctx->nameOf(ci));
                preplace_unique(ci);
            } else {
                log_info("      Cell '%s' has BEL. preplace_unique() bypassed.\n", ctx->nameOf(ci));
            }
        } else {
            // Uncomment if you want to log every single cell that isn't BUFR/BUFIO
            // log_info("    Cell '%s' is not BUFR/BUFIO.\n", ctx->nameOf(ci));
        }
    }
    log_info("Completed main global buffer preplacement loop.\n");
    log_info("Exiting XC7Packer::pack_gbs().\n");
}



void XC7Packer::pack_clocking()
{
    pack_plls();
    pack_gbs();



    // Nathan: BUFIO/BUFR use hardwired silicon ppips absent from chipdb.
    // Disconnect all ports after placement so router never attempts these arcs.
    // Mirrors ISERDESE2 cascade second-pass pattern in pack_io_xc7.cc.
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_BUFIO_BUFIO) {
            ci->disconnectPort(id_I);
            ci->disconnectPort(id_O);
        }
        if (ci->type == id_BUFR_BUFR) {
            ci->disconnectPort(id_I);
        }
    }

}

NEXTPNR_NAMESPACE_END
