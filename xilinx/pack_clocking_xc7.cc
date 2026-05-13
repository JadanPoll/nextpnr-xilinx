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
    log_info("Packing global buffers...\n");
    dict<IdString, XFormRule> gb_rules;
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;
    gb_rules[id_BUFGCTRL].new_type = id_BUFGCTRL;

    generic_xform(gb_rules);

    // Make sure prerequisites are set up first
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_PS7_PS7)
            preplace_unique(ci);
    }

    // Preplace global buffers to make use of dedicated/short routing
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_BUFGCTRL)
            try_preplace(ci, id_I0);
        if (ci->type == id_BUFG_BUFG)
            try_preplace(ci, id_I);

        // Nathan: Preplace BUFR near its clock source via dedicated routing.
        // try_preplace traverses I port driver → finds nearest BUFR BEL via short route.
        // Confirmed: Vivado places BUFR at BUFR_X0Y1 adjacent to HCLK_IOI3 tile.
        if (ci->type == id_BUFR_BUFR || ci->type == id_BUFIO_BUFIO) {
            log_info("    BUFR/BUFIO '%s' type='%s' has_bel=%d bel='%s'\n",
                ctx->nameOf(ci), ci->type.c_str(ctx),
                ci->attrs.count(id_BEL) ? 1 : 0,
                ci->attrs.count(id_BEL) ? ci->attrs.at(id_BEL).as_string().c_str() : "none");

            if (!ci->attrs.count(id_BEL)) {
                NetInfo *n = ci->getPort(id_I);
                if (n && n->driver.cell) {
                    CellInfo *drv = n->driver.cell;
                    log_info("    BUFR '%s': driver cell '%s' type '%s'\n",
                        ctx->nameOf(ci), ctx->nameOf(drv), drv->type.c_str(ctx));

                    CellInfo *pad_cell = nullptr;
                    NetInfo *pad_net = drv->getPort(id_PAD);
                    if (pad_net && pad_net->driver.cell && pad_net->driver.cell->attrs.count(id_BEL))
                        pad_cell = pad_net->driver.cell;
                    if (!pad_cell && drv->attrs.count(id_BEL))
                        pad_cell = drv;

                    if (!pad_cell) {
                        log_info("    BUFR '%s': no pad_cell found, falling back to preplace_unique\n",
                            ctx->nameOf(ci));
                    } else {
                        std::string pad_site = pad_cell->attrs.at(id_BEL).as_string();
                        log_info("    BUFR '%s': pad_cell '%s' bel '%s'\n",
                            ctx->nameOf(ci), ctx->nameOf(pad_cell), pad_site.c_str());

                        // Diagnostic logging for tile coordinates
                        BelId pad_bel_tmp = ctx->getBelByNameStr(pad_site);
                        int pad_tile_y = pad_bel_tmp.tile / ctx->chip_info->width;
                        log_info("    BUFR '%s': pad_bel_tile index=%d tile_y=%d\n",
                            ctx->nameOf(ci), pad_bel_tmp.tile, pad_tile_y);

                        auto y_pos = pad_site.find('Y');
                        if (y_pos != std::string::npos) {
                            int site_y = std::stoi(pad_site.substr(y_pos + 1,
                                pad_site.find('/', y_pos) - y_pos - 1));
                            std::string site_y_str = "Y" + std::to_string(site_y) + "/";
                            log_info("    BUFR '%s': site_y=%d searching for BUFR with I-wire containing '%s'\n",
                                ctx->nameOf(ci), site_y, site_y_str.c_str());
                            
                            for (auto bel : ctx->getBels()) {
                                if (ctx->getBelType(bel) != ci->type) continue;
                                if (used_bels.count(bel)) continue;
                                
                                int candidate_tile_y = bel.tile / ctx->chip_info->width;
                                
                                WireId i_wire = ctx->getBelPinWire(bel, id_I);
                                if (i_wire == WireId()) continue;
                                std::string iw = std::string(ctx->nameOfWire(i_wire));
                                
                                // Dump every candidate's I-wire and tile info before the filter drops it
                                log_info("    BUFR candidate bel '%s' i_wire '%s' tile=%d tile_y=%d\n",
                                    ctx->nameOfBel(bel), iw.c_str(), bel.tile, candidate_tile_y);
                                
                                // Must be in same HCLK_IOI3 tile as the IOB (tile Y matches IOB site Y)
                                if (iw.find(site_y_str) == std::string::npos) continue;
                                
                                // Within that tile, pick the BEL whose wire ends in (bel_y+2)%4
                                // bel_y extracted from BEL name e.g. BUFR_X0Y1 -> bel_y=1
                                std::string bel_name = std::string(ctx->nameOfBel(bel));
                                auto by_pos = bel_name.rfind('Y');
                                if (by_pos == std::string::npos) continue;
                                
                                int bel_y = std::stoi(bel_name.substr(by_pos + 1));
                                int expected_suffix = (bel_y + 2) % 4;
                                
                                if (iw.back() != ('0' + expected_suffix)) continue;
                                
                                used_bels.insert(bel);
                                ci->attrs[id_BEL] = std::string(ctx->nameOfBel(bel));
                                log_info("    Constrained %s '%s' to bel '%s' bel_y=%d suffix=%d\n",
                                    ci->type.c_str(ctx), ctx->nameOf(ci),
                                    ctx->nameOfBel(bel), bel_y, expected_suffix);
                                break;
                            }
                        }
                    }
                }
            }
            if (!ci->attrs.count(id_BEL))
                preplace_unique(ci);
        }
    }
}



void XC7Packer::pack_clocking()
{
    pack_plls();
    pack_gbs();
}

NEXTPNR_NAMESPACE_END
