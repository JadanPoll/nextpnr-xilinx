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
#include <boost/algorithm/string.hpp>
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

CellInfo *XC7Packer::insert_ibuf(IdString name, IdString type, NetInfo *i, NetInfo *o)
{
    auto inbuf = create_cell(ctx, type, name);
    inbuf->connectPort(id_I, i);
    inbuf->connectPort(id_O, o);
    CellInfo *inbuf_ptr = inbuf.get();
    new_cells.push_back(std::move(inbuf));
    return inbuf_ptr;
}

CellInfo *XC7Packer::insert_diffibuf(IdString name, IdString type, const std::array<NetInfo *, 2> &i, NetInfo *o)
{
    auto inbuf = create_cell(ctx, type, name);
    inbuf->connectPort(id_I, i[0]);
    inbuf->connectPort(id_IB, i[1]);
    inbuf->connectPort(id_O, o);
    CellInfo *inbuf_ptr = inbuf.get();
    new_cells.push_back(std::move(inbuf));
    return inbuf_ptr;
}

std::string get_tilename_by_sitename(Context *ctx, std::string site)
{
    if (ctx->site_by_name.count(site)) {
        int tile, siteid;
        std::tie(tile, siteid) = ctx->site_by_name.at(site);
        return ctx->chip_info->tile_insts[tile].name.get();
    }
    return std::string();
}

void XC7Packer::decompose_iob(CellInfo *xil_iob, bool is_hr, const std::string &iostandard)
{
    bool is_se_ibuf = xil_iob->type.in(id_IBUF, id_IBUF_IBUFDISABLE, id_IBUF_INTERMDISABLE);
    bool is_se_iobuf = xil_iob->type.in(id_IOBUF, id_IOBUF_DCIEN, id_IOBUF_INTERMDISABLE);
    bool is_se_obuf = xil_iob->type.in(id_OBUF, id_OBUFT);

    auto pad_site = [&](NetInfo *n) {
        for (auto user : n->users)
            if (user.cell->type == id_PAD)
                return ctx->getBelSite(ctx->getBelByNameStr(user.cell->attrs[id_BEL].as_string()));
        NPNR_ASSERT_FALSE(("can't find PAD for net " + n->name.str(ctx)).c_str());
    };

    /*
     * IO primitives in Xilinx are complex "macros" that usually expand to more than one BEL
     * To avoid various nasty bugs (such as auto-transformation by Vivado of dedicated INV primitives to LUT1s), we
     * have to maintain this hierarchy so it can be re-built during DCP conversion in RapidWright
     */
    dict<IdString, PortInfo> orig_ports = xil_iob->ports;
    std::vector<CellInfo *> subcells;

    if (is_se_ibuf || is_se_iobuf) {
        log_info("Generating input buffer for '%s'\n", xil_iob->name.c_str(ctx));
        NetInfo *pad_net = xil_iob->getPort(is_se_iobuf ? id_IO : id_I);
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        if (!is_se_iobuf)
            xil_iob->disconnectPort(id_I);

        NetInfo *top_out = xil_iob->getPort(id_O);
        xil_iob->disconnectPort(id_O);

        IdString ibuf_type = id_IBUF;
        if (xil_iob->type.in(id_IBUF_IBUFDISABLE, id_IOBUF_DCIEN))
            ibuf_type = id_IBUF_IBUFDISABLE;
        if (xil_iob->type.in(id_IBUF_INTERMDISABLE, id_IOBUF_INTERMDISABLE))
            ibuf_type = id_IBUF_INTERMDISABLE;

        CellInfo *inbuf = insert_ibuf(int_name(xil_iob->name, "IBUF", is_se_iobuf), ibuf_type, pad_net, top_out);
        std::string tile = get_tilename_by_sitename(ctx, site);
        if (boost::starts_with(tile, "RIOB18_"))
            inbuf->attrs[id_BEL] = site + "/IOB18/INBUF_DCIEN";
        else
            inbuf->attrs[id_BEL] = site + "/IOB33/INBUF_EN";
        xil_iob->movePortTo(id_IBUFDISABLE, inbuf, id_IBUFDISABLE);
        xil_iob->movePortTo(id_INTERMDISABLE, inbuf, id_INTERMDISABLE);

        if (is_se_iobuf)
            subcells.push_back(inbuf);
    }

    if (is_se_obuf || is_se_iobuf) {
        log_info("Generating output buffer for '%s'\n", xil_iob->name.c_str(ctx));
        NetInfo *pad_net = xil_iob->getPort(is_se_iobuf ? id_IO : id_O);
        NPNR_ASSERT(pad_net != nullptr);
        std::string site = pad_site(pad_net);
        xil_iob->disconnectPort(is_se_iobuf ? id_IO : id_O);
        bool has_dci = xil_iob->type == id_IOBUF_DCIEN;
        CellInfo *obuf = insert_obuf(
                int_name(xil_iob->name, (is_se_iobuf || xil_iob->type == id_OBUFT) ? "OBUFT" : "OBUF", !is_se_obuf),
                is_se_iobuf ? (has_dci ? id_OBUFT_DCIEN : id_OBUFT) : xil_iob->type, xil_iob->getPort(id_I), pad_net,
                xil_iob->getPort(id_T));
        std::string tile = get_tilename_by_sitename(ctx, site);
        if (boost::starts_with(tile, "RIOB18_"))
            obuf->attrs[id_BEL] = site + "/IOB18/OUTBUF_DCIEN";
        else
            obuf->attrs[id_BEL] = site + "/IOB33/OUTBUF";
        xil_iob->movePortTo(id_DCITERMDISABLE, obuf, id_DCITERMDISABLE);
        if (is_se_iobuf)
            subcells.push_back(obuf);
    }

    bool is_diff_ibuf = xil_iob->type.in(id_IBUFDS, id_IBUFDS_INTERMDISABLE, id_IBUFDS);
    bool is_diff_iobuf = xil_iob->type.in(id_IOBUFDS, id_IOBUFDS_DCIEN);
    bool is_diff_out_iobuf =
            xil_iob->type.in(id_IOBUFDS_DIFF_OUT, id_IOBUFDS_DIFF_OUT_DCIEN, id_IOBUFDS_DIFF_OUT_INTERMDISABLE);
    bool is_diff_obuf = xil_iob->type.in(id_OBUFDS, id_OBUFTDS);

    if (is_diff_ibuf || is_diff_iobuf) {
        NetInfo *pad_p_net = xil_iob->getPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IO : id_I);
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net = xil_iob->getPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IOB : id_IB);
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_n_net);
        std::string tile_p = get_tilename_by_sitename(ctx, site_p);
        bool is_riob18 = boost::starts_with(tile_p, "RIOB18_");

        if (!is_diff_iobuf && !is_diff_out_iobuf) {
            xil_iob->disconnectPort(id_I);
            xil_iob->disconnectPort(id_IB);
        }

        NetInfo *top_out = xil_iob->getPort(id_O);
        xil_iob->disconnectPort(id_O);

        IdString ibuf_type = id_IBUFDS;
        CellInfo *inbuf = insert_diffibuf(int_name(xil_iob->name, "IBUF", is_se_iobuf), ibuf_type,
                                          {pad_p_net, pad_n_net}, top_out);
        if (is_riob18) {
            inbuf->attrs[id_BEL] = site_p + "/IOB18/INBUF_DCIEN";
            inbuf->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB18");
        } else {
            inbuf->attrs[id_BEL] = site_p + "/IOB33/INBUF_EN";
            inbuf->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB33");
        }

        if (is_diff_iobuf)
            subcells.push_back(inbuf);
    }


    // Nathan: xc7 HR IOB33 differential output fix.
    // O_ININV does not exist as a wire or pip in LIOB33 tile DB (confirmed via
    // prjxray-db/spartan7/tile_type_LIOB33.json: zero O_ININV wires or pips).
    // Attempting to route through O_ININV causes: "Failed to route arc... O_ININV_OUT
    // to OUSED_OUT" — confirmed from nextpnr error on OBUFDS sanity test.
    //
    // xc7 HR differential output works via hardware pip DIFFO_OUT0->DIFFO_IN1
    // (confirmed in tile DB: 'LIOB33.IOB_DIFFO_OUT0->IOB_DIFFO_IN1' pip exists).
    // The OUT_DIFF config bit in fasm.cc enables this pip for DIFF_* iostandards.
    // For LVDS_25/TMDS_33, the differential drive is handled by the LVDS_25.OUT
    // config bit on the master side — no OUT_DIFF or inv needed.
    //
    // IOB18 HP banks (is_riob18) DO support O_ININV routing — left unchanged.
    // Only xc7 HR IOB33 banks are affected by this fix.


    if (is_diff_obuf || is_diff_out_iobuf || is_diff_iobuf) {
        // FIXME: true diff outputs
        NetInfo *pad_p_net = xil_iob->getPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IO : id_O);
        NPNR_ASSERT(pad_p_net != nullptr);
        std::string site_p = pad_site(pad_p_net);
        NetInfo *pad_n_net = xil_iob->getPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IOB : id_OB);
        NPNR_ASSERT(pad_n_net != nullptr);
        std::string site_n = pad_site(pad_n_net);
        std::string tile_p = get_tilename_by_sitename(ctx, site_p);
        bool is_riob18 = boost::starts_with(tile_p, "RIOB18_");
        xil_iob->disconnectPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IO : id_O);
        xil_iob->disconnectPort((is_diff_iobuf || is_diff_out_iobuf) ? id_IOB : id_OB);
        bool has_dci = xil_iob->type.in(id_IOBUFDS_DCIEN, id_IOBUFDSE3);
        CellInfo *obuf_p = insert_obuf(int_name(xil_iob->name, is_diff_obuf ? "P" : "OBUFTDS$subcell$P"),
                                       (is_diff_iobuf || is_diff_out_iobuf || (xil_iob->type == id_OBUFTDS))
                                               ? (has_dci ? id_OBUFT_DCIEN : id_OBUFT)
                                               : id_OBUF,
                                       xil_iob->getPort(id_I), pad_p_net, xil_iob->getPort(id_T));
        if (is_riob18) {
            obuf_p->attrs[id_BEL] = site_p + "/IOB18/OUTBUF_DCIEN";
            obuf_p->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB18");
        } else {
            obuf_p->attrs[id_BEL] = site_p + "/IOB33/OUTBUF";
            obuf_p->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB33");
        }
        subcells.push_back(obuf_p);
        obuf_p->connectPort(id_DCITERMDISABLE, xil_iob->getPort(id_DCITERMDISABLE));
        if (is_riob18) {
            // Nathan: IOB18 HP banks use explicit inverter + slave outbuf.
            // O_ININV is routable on HP banks.
            NetInfo *inv_i = create_internal_net(xil_iob->name, is_diff_obuf ? "I_B" : "OBUFTDS$subnet$I_B");
            CellInfo *inv = insert_outinv(int_name(xil_iob->name, is_diff_obuf ? "INV" : "OBUFTDS$subcell$INV"),
                                          xil_iob->getPort(id_I), inv_i);
            inv->attrs[id_BEL] = site_n + "/IOB18S/O_ININV";
            inv->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB18S");
            CellInfo *obuf_n = insert_obuf(int_name(xil_iob->name, is_diff_obuf ? "N" : "OBUFTDS$subcell$N"),
                                           (is_diff_iobuf || is_diff_out_iobuf || (xil_iob->type == id_OBUFTDS))
                                                   ? (has_dci ? id_OBUFT_DCIEN : id_OBUFT)
                                                   : id_OBUF,
                                           inv_i, pad_n_net, xil_iob->getPort(id_T));
            obuf_n->attrs[id_BEL] = site_n + "/IOB18/OUTBUF_DCIEN";
            obuf_n->attrs[id_X_IOB_SITE_TYPE] = std::string("IOB18S");
            obuf_n->connectPort(id_DCITERMDISABLE, xil_iob->getPort(id_DCITERMDISABLE));
            subcells.push_back(inv);
            subcells.push_back(obuf_n);
        }
        // Nathan: xc7 HR IOB33 — no inv or obuf_n needed.
        // N-side driven by hardware pip DIFFO_OUT0->DIFFO_IN1 (confirmed in prjxray DB).
        // OUT_DIFF config bit (fasm.cc) enables this pip for DIFF_* iostandards.
        // O_ININV has no wires/pips in LIOB33 tile — confirmed not routable.
        xil_iob->disconnectPort(id_DCITERMDISABLE);
        subcells.push_back(obuf_p);
    }



    if (!subcells.empty()) {
        for (auto sc : subcells) {
            sc->attrs[id_X_ORIG_MACRO_PRIM] = xil_iob->type.str(ctx);
            for (auto &p : sc->ports) {
                std::string macro_ports;
                for (auto &orig : orig_ports) {
                    if ((orig.second.net != nullptr) && (orig.second.net == p.second.net)) {
                        macro_ports += orig.first.str(ctx);
                        macro_ports += ',';
                        macro_ports += (orig.second.type == PORT_INOUT) ? "inout"
                                       : (orig.second.type == PORT_OUT) ? "out"
                                                                        : "in";
                        macro_ports += ";";
                    }
                }
                if (!macro_ports.empty()) {
                    macro_ports.erase(macro_ports.size() - 1);
                    sc->attrs[ctx->id("X_MACRO_PORTS_" + p.first.str(ctx))] = macro_ports;
                }
            }
        }
    }
}

void XC7Packer::pack_io()
{
    // make sure the supporting data structure of
    // get_tilename_by_sitename()
    // is initialized before we use it below
    ctx->setup_byname();

    log_info("Inserting IO buffers..\n");

    get_top_level_pins(ctx, toplevel_ports);
    // Insert PAD cells on top level IO, and IO buffers where one doesn't exist already
    std::vector<std::pair<CellInfo *, PortRef>> pad_and_buf;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == ctx->id("$nextpnr_ibuf") || ci->type == ctx->id("$nextpnr_iobuf") ||
            ci->type == ctx->id("$nextpnr_obuf"))
            pad_and_buf.push_back(insert_pad_and_buf(ci));
    }
    flush_cells();
    pool<BelId> used_io_bels;
    int unconstr_io_count = 0;
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        // Process location constraints
        if (pad->attrs.count(id_PACKAGE_PIN)) {
            pad->attrs[id_LOC] = pad->attrs.at(id_PACKAGE_PIN);
        }
        if (pad->attrs.count(id_LOC)) {
            std::string loc = pad->attrs.at(id_LOC).to_string();
            std::string site = ctx->getPackagePinSite(loc);
            if (site.empty())
                log_error("Unable to constrain IO '%s', device does not have a pin named '%s'\n", pad->name.c_str(ctx),
                          loc.c_str());
            log_info("    Constraining '%s' to site '%s'\n", pad->name.c_str(ctx), site.c_str());
            std::string tile = get_tilename_by_sitename(ctx, site);
            log_info("    Tile '%s'\n", tile.c_str());
            if (boost::starts_with(tile, "RIOB18_"))
                pad->attrs[id_BEL] = std::string(site + "/IOB18/PAD");
            else
                pad->attrs[id_BEL] = std::string(site + "/IOB33/PAD");
        }
        if (pad->attrs.count(id_BEL)) {
            used_io_bels.insert(ctx->getBelByNameStr(pad->attrs.at(id_BEL).as_string()));
        } else {
            ++unconstr_io_count;
        }
    }
    std::queue<BelId> available_io_bels;
    IdString pad_id = ctx->xc7 ? id_PAD : id_IOB_PAD;
    for (auto bel : ctx->getBels()) {
        if (int(available_io_bels.size()) >= unconstr_io_count)
            break;
        if (ctx->locInfo(bel).bel_data[bel.index].site_variant != 0)
            continue;
        if (ctx->getBelType(bel) != pad_id)
            continue;
        if (ctx->getBelPackagePin(bel) == ".")
            continue;
        if (used_io_bels.count(bel))
            continue;
        available_io_bels.push(bel);
    }
    int avail_count = int(available_io_bels.size());
    // Constrain unconstrained IO
    for (auto &iob : pad_and_buf) {
        CellInfo *pad = iob.first;
        if (!pad->attrs.count(id_BEL)) {
            if (available_io_bels.empty()) {
                log_error("IO placer ran out of available IOs (%d available IO, %d unconstrained pins)\n", avail_count,
                          unconstr_io_count);
            }
            pad->attrs[id_BEL] = std::string(ctx->nameOfBel(available_io_bels.front()));
            available_io_bels.pop();
        }
    }
    // Decompose macro IO primitives to smaller primitives that map logically to the actual IO Bels
    for (auto &iob : pad_and_buf) {
        if (packed_cells.count(iob.second.cell->name))
            continue;
        decompose_iob(iob.second.cell, true, str_or_default(iob.first->attrs, id_IOSTANDARD, ""));
        packed_cells.insert(iob.second.cell->name);
    }
    flush_cells();

    dict<IdString, XFormRule> hriobuf_rules, hpiobuf_rules;
    hriobuf_rules[id_OBUF].new_type = id_IOB33_OUTBUF;
    hriobuf_rules[id_OBUF].port_xform[id_I] = id_IN;
    hriobuf_rules[id_OBUF].port_xform[id_O] = id_OUT;
    hriobuf_rules[id_OBUF].port_xform[id_T] = id_TRI;
    hriobuf_rules[id_OBUFT] = XFormRule(hriobuf_rules[id_OBUF]);

    hriobuf_rules[id_IBUF].new_type = id_IOB33_INBUF_EN;
    hriobuf_rules[id_IBUF].port_xform[id_I] = id_PAD;
    hriobuf_rules[id_IBUF].port_xform[id_O] = id_OUT;
    hriobuf_rules[id_IBUF_INTERMDISABLE] = XFormRule(hriobuf_rules[id_IBUF]);
    hriobuf_rules[id_IBUF_IBUFDISABLE] = XFormRule(hriobuf_rules[id_IBUF]);
    hriobuf_rules[id_IBUFDS_INTERMDISABLE_INT] = XFormRule(hriobuf_rules[id_IBUF]);
    hriobuf_rules[id_IBUFDS_INTERMDISABLE_INT].port_xform[id_IB] = id_DIFFI_IN;
    hriobuf_rules[id_IBUFDS] = XFormRule(hriobuf_rules[id_IBUF]);
    hriobuf_rules[id_IBUFDS].port_xform[id_IB] = id_DIFFI_IN;

    hpiobuf_rules[id_OBUF].new_type = id_IOB18_OUTBUF_DCIEN;
    hpiobuf_rules[id_OBUF].port_xform[id_I] = id_IN;
    hpiobuf_rules[id_OBUF].port_xform[id_O] = id_OUT;
    hpiobuf_rules[id_OBUF].port_xform[id_T] = id_TRI;
    hpiobuf_rules[id_OBUFT] = XFormRule(hpiobuf_rules[id_OBUF]);

    hpiobuf_rules[id_IBUF].new_type = id_IOB18_INBUF_DCIEN;
    hpiobuf_rules[id_IBUF].port_xform[id_I] = id_PAD;
    hpiobuf_rules[id_IBUF].port_xform[id_O] = id_OUT;
    hpiobuf_rules[id_IBUF_INTERMDISABLE] = XFormRule(hpiobuf_rules[id_IBUF]);
    hpiobuf_rules[id_IBUF_IBUFDISABLE] = XFormRule(hpiobuf_rules[id_IBUF]);
    hriobuf_rules[id_IBUFDS_INTERMDISABLE_INT] = XFormRule(hriobuf_rules[id_IBUF]);
    hpiobuf_rules[id_IBUFDS_INTERMDISABLE_INT].port_xform[id_IB] = id_DIFFI_IN;
    hpiobuf_rules[id_IBUFDS] = XFormRule(hpiobuf_rules[id_IBUF]);
    hpiobuf_rules[id_IBUFDS].port_xform[id_IB] = id_DIFFI_IN;

    // Special xform for OBUFx and IBUFx.
    dict<IdString, XFormRule> rules;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (!ci->attrs.count(id_BEL))
            continue;
        std::string belname = ci->attrs[id_BEL].c_str();
        size_t pos = belname.find("/");
        if (belname.substr(pos + 1, 5) == "IOB18")
            rules = hpiobuf_rules;
        else if (belname.substr(pos + 1, 5) == "IOB33")
            rules = hriobuf_rules;
        else
            log_error("Unexpected IOBUF BEL %s\n", belname.c_str());
        if (rules.count(ci->type)) {
            xform_cell(rules, ci);
        }
    }

    dict<IdString, XFormRule> hrio_rules;
    hrio_rules[id_PAD].new_type = id_PAD;

    hrio_rules[id_INV].new_type = id_INVERTER;
    hrio_rules[id_INV].port_xform[id_I] = id_IN;
    hrio_rules[id_INV].port_xform[id_O] = id_OUT;

    hrio_rules[id_PS7].new_type = id_PS7_PS7;

    generic_xform(hrio_rules, true);

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        std::string type = ci->type.str(ctx);
        if (!boost::starts_with(type, "IOB33") && !boost::starts_with(type, "IOB18"))
            continue;
        if (!ci->attrs.count(id_X_IOB_SITE_TYPE))
            continue;
        type.replace(0, 5, ci->attrs.at(id_X_IOB_SITE_TYPE).as_string());
        ci->type = ctx->id(type);
    }

    // check all PAD cells for IOSTANDARD/DRIVE
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        std::string type = ci->type.str(ctx);
        if (type != "PAD")
            continue;
        check_valid_pad(ci, type);
    }
}

void XC7Packer::check_valid_pad(CellInfo *ci, std::string type)
{
    auto iostandard_id = id_IOSTANDARD;
    auto iostandard_attr = ci->attrs.find(iostandard_id);
    if (iostandard_attr == ci->attrs.end())
        log_error("port %s has no IOSTANDARD property", ci->name.c_str(ctx));

    auto iostandard = iostandard_attr->second.as_string();
    if (!boost::starts_with(iostandard, "LVTTL") && !boost::starts_with(iostandard, "LVCMOS"))
        return;

    auto drive_attr = ci->attrs.find(id_DRIVE);
    // no drive strength attribute: use default
    if (drive_attr == ci->attrs.end())
        return;
    auto drive = drive_attr->second.as_int64();

    bool is_iob33 = boost::starts_with(type, "IOB33");
    if (is_iob33) {
        if (drive == 4 || drive == 8 || drive == 12)
            return;
        if (iostandard != "LVCMOS12" && drive == 16)
            return;
        if ((iostandard == "LVCMOS18" || iostandard == "LVTTL") && drive == 24)
            return;
    } else { // IOB18
        if (drive == 2 || drive == 4 || drive == 6 || drive == 8)
            return;
        if (iostandard != "LVCMOS12" && (drive == 12 || drive == 16))
            return;
    }

    log_error("unsupported DRIVE strength property %s for port %s", drive_attr->second.c_str(), ci->name.c_str(ctx));
}

std::string XC7Packer::get_ologic_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB18/OUTBUF_DCIEN");
    else
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB33/OUTBUF");
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, id_IN));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "OLOGIC"))
                return site;
        }
        for (auto pip : ctx->getPipsUphill(cursor))
            visit.push(ctx->getPipSrcWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find OLOGIC");
}

std::string XC7Packer::get_ilogic_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB18/INBUF_DCIEN");
    else
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB33/INBUF_EN");
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, id_OUT));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "ILOGIC"))
                return site;
        }
        for (auto pip : ctx->getPipsDownhill(cursor))
            visit.push(ctx->getPipDstWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find ILOGIC");
}

std::string XC7Packer::get_idelay_site(const std::string &io_bel)
{
    BelId ibc_bel;
    if (boost::contains(io_bel, "IOB18"))
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB18/INBUF_DCIEN");
    else
        ibc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB33/INBUF_EN");
    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(ibc_bel, id_OUT));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "IDELAY"))
                return site;
        }
        for (auto pip : ctx->getPipsDownhill(cursor))
            visit.push(ctx->getPipDstWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find IDELAY");
}

std::string XC7Packer::get_odelay_site(const std::string &io_bel)
{
    BelId obc_bel;
    if (boost::contains(io_bel, "IOB18"))
        obc_bel = ctx->getBelByNameStr(io_bel.substr(0, io_bel.find('/')) + "/IOB18/OUTBUF_DCIEN");
    else
        log_error("BEL %s is located on a high range bank. High range banks do not have ODELAY", io_bel.c_str());

    std::queue<WireId> visit;
    visit.push(ctx->getBelPinWire(obc_bel, id_IN));

    while (!visit.empty()) {
        WireId cursor = visit.front();
        visit.pop();
        for (auto bp : ctx->getWireBelPins(cursor)) {
            std::string site = ctx->getBelSite(bp.bel);
            if (boost::starts_with(site, "ODELAY"))
                return site;
        }
        for (auto pip : ctx->getPipsUphill(cursor))
            visit.push(ctx->getPipSrcWire(pip));
    }
    NPNR_ASSERT_FALSE("failed to find ODELAY");
}

std::string XC7Packer::get_ioctrl_site(const std::string &io_bel)
{
    std::vector<std::string> parts;
    boost::split(parts, io_bel, boost::is_any_of("/"));
    auto loc = parts[0];
    auto iobank = parts[1];
    auto pad_bel_str = loc + "/" + iobank + "/PAD";
    auto msg = "could not get bel for: '" + pad_bel_str + "'";

    BelId pad_bel = ctx->getBelByNameStr(pad_bel_str);
    NPNR_ASSERT_MSG(0 <= pad_bel.tile && 0 <= pad_bel.index, msg.c_str());

    int hclk_tile = ctx->getHclkForIob(pad_bel);
    auto &td = ctx->chip_info->tile_insts[hclk_tile];
    for (int i = 0; i < td.num_sites; i++) {
        auto &sd = td.site_insts[i];
        std::string sn = sd.name.get();
        if (boost::starts_with(sn, "IDELAYCTRL"))
            return sn;
    }
    NPNR_ASSERT_FALSE("failed to find IOCTRL");
}

void XC7Packer::fold_inverter(CellInfo *cell, std::string port)
{
    IdString p = ctx->id(port);
    NetInfo *net = cell->getPort(p);
    if (net == nullptr)
        return;
    CellInfo *drv = net->driver.cell;
    if (drv == nullptr)
        return;
    if (drv->type == id_LUT1 && int_or_default(drv->params, id_INIT, 0) == 1) {
        cell->disconnectPort(p);
        NetInfo *preinv = drv->getPort(id_I0);
        cell->connectPort(p, preinv);
        cell->params[ctx->id("IS_" + port + "_INVERTED")] = 1;
        if (net->users.empty())
            packed_cells.insert(drv->name);
    } else if (drv->type == id_INV) {
        cell->disconnectPort(p);
        NetInfo *preinv = drv->getPort(id_I);
        cell->connectPort(p, preinv);
        cell->params[ctx->id("IS_" + port + "_INVERTED")] = 1;
        if (net->users.empty())
            packed_cells.insert(drv->name);
    }
}

void XC7Packer::pack_iologic()
{
    dict<IdString, BelId> iodelay_to_io;
    dict<IdString, XFormRule> iologic_rules;

    // IDDR
    iologic_rules[id_IDDR].new_type = id_ILOGICE3_IFF;
    iologic_rules[id_IDDR].port_multixform[id_C] = {id_CK, id_CKB};
    iologic_rules[id_IDDR].port_xform[id_S] = id_SR;
    iologic_rules[id_IDDR].port_xform[id_R] = id_SR;

    // SERDES
    iologic_rules[id_ISERDESE2].new_type = id_ISERDESE2_ISERDESE2;
    iologic_rules[id_OSERDESE2].new_type = id_OSERDESE2_OSERDESE2;

    // DELAY
    iologic_rules[id_IDELAYE2].new_type = id_IDELAYE2_IDELAYE2;
    iologic_rules[id_ODELAYE2].new_type = id_ODELAYE2_ODELAYE2;

    // Handles pseudo-diff output buffers without finding multiple sinks
    auto find_p_outbuf = [&](NetInfo *net) {
        CellInfo *outbuf = nullptr;
        for (auto &usr : net->users) {
            IdString type = usr.cell->type;
            if (type.in(id_IOB33_OUTBUF, id_IOB33M_OUTBUF, id_IOB18_OUTBUF_DCIEN, id_IOB18M_OUTBUF_DCIEN)) {
                if (outbuf != nullptr)
                    return (CellInfo *)nullptr; // drives multiple outputs
                outbuf = usr.cell;
            } else if (type == id_ODELAYE2) {
                auto dataout = usr.cell->ports.find(id_DATAOUT);
                if (dataout != usr.cell->ports.end()) {
                    for (auto &user : dataout->second.net->users) {
                        IdString dataout_type = user.cell->type;
                        if (dataout_type.in(id_IOB18_OUTBUF_DCIEN, id_IOB18M_OUTBUF_DCIEN)) {
                            if (outbuf != nullptr)
                                return (CellInfo *)nullptr; // drives multiple outputs
                            outbuf = user.cell;
                        }
                    }
                } else {
                    if (outbuf != nullptr)
                        return (CellInfo *)nullptr; // drives multiple outputs
                }
            }
        }
        return outbuf;
    };

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_IDELAYE2) {
            NetInfo *d = ci->getPort(id_IDATAIN);
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected IDATAIN input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            BelId io_bel;
            if (boost::contains(drv->type.str(ctx), "INBUF_EN") || boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                io_bel = ctx->getBelByNameStr(drv->attrs.at(id_BEL).as_string());
            else
                log_error("%s '%s' has IDATAIN input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));
            std::string iol_site = get_idelay_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[id_BEL] = iol_site + "/IDELAYE2";
            ci->attrs[id_X_IO_BEL] = ctx->getBelName(io_bel).str(ctx);
            iodelay_to_io[ci->name] = io_bel;
        } else if (ci->type == id_ODELAYE2) {
            NetInfo *dataout = ci->getPort(id_DATAOUT);
            if (dataout == nullptr || dataout->users.empty())
                log_error("%s '%s' has disconnected DATAOUT input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            auto no_users = dataout->users.entries();
            for (auto userport : dataout->users) {
                CellInfo *user = userport.cell;
                auto user_type = user->type.str(ctx);
                // OBUFDS has the negative pin connected to an inverter
                if (no_users == 2 && user_type == "INVERTER")
                    continue;
                if (boost::contains(user_type, "OUTBUF_EN") || boost::contains(user_type, "OUTBUF_DCIEN"))
                    io_bel = ctx->getBelByNameStr(user->attrs.at(id_BEL).as_string());
                else
                    // TODO: support SIGNAL_PATTERN = CLOCK
                    log_error("%s '%s' has DATAOUT connected to unsupported cell type %s\n", ci->type.c_str(ctx),
                              ctx->nameOf(ci), user_type.c_str());
            }
            std::string iol_site = get_odelay_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[id_BEL] = iol_site + "/ODELAYE2";
            ci->attrs[id_X_IO_BEL] = ctx->getBelName(io_bel).str(ctx);
            iodelay_to_io[ci->name] = io_bel;
        }
    }

    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_ODDR) {
            NetInfo *q = ci->getPort(id_Q);
            if (q == nullptr || q->users.empty())
                log_error("%s '%s' has disconnected Q output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            BelId io_bel;
            CellInfo *ob = find_p_outbuf(q);
            if (ob != nullptr)
                io_bel = ctx->getBelByNameStr(ob->attrs.at(id_BEL).as_string());
            else
                log_error("%s '%s' has illegal fanout on Q output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            auto io_bel_str = ctx->getBelName(io_bel).str(ctx);
            std::string ol_site = get_ologic_site(io_bel_str);

            PortRef dest_port = *q->users.begin();
            auto is_tristate = dest_port.port == id_TRI;

            dict<IdString, XFormRule> oddr_rules;
            if (boost::contains(io_bel_str, "IOB18"))
                oddr_rules[id_ODDR].new_type = is_tristate ? id_OLOGICE2_TFF : id_OLOGICE2_OUTFF;
            else
                oddr_rules[id_ODDR].new_type = is_tristate ? id_OLOGICE3_TFF : id_OLOGICE3_OUTFF;
            oddr_rules[id_ODDR].port_xform[id_C] = id_CK;
            oddr_rules[id_ODDR].port_xform[id_S] = id_SR;
            oddr_rules[id_ODDR].port_xform[id_R] = id_SR;
            xform_cell(oddr_rules, ci);

            ci->attrs[id_BEL] = ol_site + (is_tristate ? "/TFF" : "/OUTFF");
        } else if (ci->type == id_OSERDESE2) {
            bool is_slave = str_or_default(ci->params, id_SERDES_MODE, "MASTER") == "SLAVE";
            if (is_slave) {
                // Find MASTER via SHIFTOUT1 net
                NetInfo *shiftout = ci->getPort(ctx->id("SHIFTOUT1"));
                if (shiftout == nullptr || shiftout->users.empty())
                    log_error("OSERDESE2 SLAVE '%s' has disconnected SHIFTOUT1\n", ctx->nameOf(ci));
                
                CellInfo *master = nullptr;
                for (auto &usr : shiftout->users)
                    if (usr.cell != ci) { master = usr.cell; break; }
                if (master == nullptr)
                    log_error("OSERDESE2 SLAVE '%s' cannot find MASTER\n", ctx->nameOf(ci));

                NetInfo *master_oq = master->getPort(id_OQ);
                if (master_oq == nullptr || master_oq->users.empty())
                    log_error("OSERDESE2 MASTER for SLAVE '%s' has disconnected OQ\n", ctx->nameOf(ci));

                CellInfo *ob = find_p_outbuf(master_oq);
                if (ob == nullptr)
                    log_error("OSERDESE2 MASTER for SLAVE '%s' has illegal OQ fanout\n", ctx->nameOf(ci));

                // SAFETY: Ensure the OBUF actually has a placement constraint
                if (ob->attrs.count(id_BEL) == 0)
                    log_error("OBUF '%s' for OSERDESE2 MASTER is missing a LOC constraint\n", ctx->nameOf(ob));

                std::string master_site = get_ologic_site(ob->attrs.at(id_BEL).as_string());
                size_t y_pos = master_site.rfind('Y');
                int master_y = std::stoi(master_site.substr(y_pos + 1));
                
                // SAFETY: Prevent NextPNR from crashing if Master is at the bottom of a column
                if (master_y == 0)
                    log_error("OSERDESE2 MASTER at '%s' is at Y0; cannot place SLAVE at Y-1\n", master_site.c_str());

                std::string slave_site = master_site.substr(0, y_pos + 1) + std::to_string(master_y - 1);
                ci->attrs[id_BEL] = slave_site + "/OSERDESE2";
            } 
            
            
            
            else {
                NetInfo *q = ci->getPort(id_OQ);
                NetInfo *ofb = ci->getPort(id_OFB);
                bool q_disconnected = q == nullptr || q->users.empty();
                bool ofb_disconnected = ofb == nullptr || ofb->users.empty();
                if (q_disconnected && ofb_disconnected)
                    log_error("%s '%s' has disconnected OQ/OFB output ports\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                
                BelId io_bel;
                CellInfo *ob = !q_disconnected ? find_p_outbuf(q) : find_p_outbuf(ofb);
                if (ob != nullptr && ob->attrs.count(id_BEL)) {
                    io_bel = ctx->getBelByNameStr(ob->attrs.at(id_BEL).as_string());
                } else {
                    log_error("%s '%s' has illegal fanout or unconstrained output\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                }
                std::string ol_site = get_ologic_site(ctx->getBelName(io_bel).str(ctx));
                ci->attrs[id_BEL] = ol_site + "/OSERDESE2";
            }
        }

        
        else if (ci->type == id_ISERDESE2) {
            bool is_slave = str_or_default(ci->params, id_SERDES_MODE, "MASTER") == "SLAVE";
            if (is_slave) {
                // Nathan: ISERDESE2 SLAVE cascade. Data flows MASTER->SLAVE via SHIFTOUT1->SHIFTIN1.
                // SLAVE site = MASTER site Y-1. Confirmed: ILOGIC_X0Y87(SLAVE)+ILOGIC_X0Y88(MASTER) share tile.
                NetInfo *shiftin = ci->getPort(ctx->id("SHIFTIN1"));
                if (shiftin == nullptr || shiftin->driver.cell == nullptr)
                    log_error("ISERDESE2 SLAVE '%s' has disconnected SHIFTIN1\n", ctx->nameOf(ci));
                
                CellInfo *master = shiftin->driver.cell;
                std::string iobdelay = str_or_default(master->params, id_IOBDELAY, "NONE");
                BelId master_io_bel;
                
                // We use the MASTER's input to find the physical tile
                if (iobdelay == "IFD") {
                    NetInfo *d = master->getPort(id_DDLY);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("MASTER ISERDESE2 '%s' has disconnected DDLY\n", ctx->nameOf(master));
                    master_io_bel = iodelay_to_io.at(d->driver.cell->name);
                } else {
                    NetInfo *d = master->getPort(id_D);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("MASTER ISERDESE2 '%s' has disconnected D\n", ctx->nameOf(master));
                    
                    // SAFETY: Prevent crashing if fuzzer generates an unconstrained Master INBUF
                    if (d->driver.cell->attrs.count(id_BEL) == 0)
                        log_error("INBUF for MASTER ISERDESE2 '%s' is missing a LOC constraint\n", ctx->nameOf(master));
                        
                    master_io_bel = ctx->getBelByNameStr(d->driver.cell->attrs.at(id_BEL).as_string());
                }
                
                std::string master_site = get_ilogic_site(ctx->getBelName(master_io_bel).str(ctx));
                size_t y_pos = master_site.rfind('Y');
                int master_y = std::stoi(master_site.substr(y_pos + 1));
                
                // SAFETY: Prevent NextPNR from crashing if Master is at the bottom of a column
                if (master_y == 0)
                    log_error("ISERDESE2 MASTER at '%s' is at Y0; cannot place SLAVE at Y-1\n", master_site.c_str());
                    
                std::string slave_site = master_site.substr(0, y_pos + 1) + std::to_string(master_y - 1);
                ci->attrs[id_BEL] = slave_site + "/ISERDESE2";
                
            } else {
                // MASTER or STANDALONE logic (Original NextPNR code goes here)
                fold_inverter(ci, "CLKB");
                fold_inverter(ci, "OCLKB");
                std::string iobdelay = str_or_default(ci->params, id_IOBDELAY, "NONE");
                BelId io_bel;
                if (iobdelay == "IFD") {
                    NetInfo *d = ci->getPort(id_DDLY);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("%s '%s' has disconnected DDLY input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                    CellInfo *drv = d->driver.cell;
                    if (boost::contains(drv->type.str(ctx), "IDELAYE2") && d->driver.port == id_DATAOUT)
                        io_bel = iodelay_to_io.at(drv->name);
                    else
                        log_error("%s '%s' has DDLY input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                                  ctx->nameOf(ci), drv->type.c_str(ctx));
                } else if (iobdelay == "NONE") {
                    NetInfo *d = ci->getPort(id_D);
                    if (d == nullptr || d->driver.cell == nullptr)
                        log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                    CellInfo *drv = d->driver.cell;
                    if (boost::contains(drv->type.str(ctx), "INBUF_EN") ||
                        boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                        io_bel = ctx->getBelByNameStr(drv->attrs.at(id_BEL).as_string());
                    else
                        log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                                  ctx->nameOf(ci), drv->type.c_str(ctx));
                } else {
                    log_error("%s '%s' has unsupported IOBDELAY value '%s'\n", ci->type.c_str(ctx), ctx->nameOf(ci),
                              iobdelay.c_str());
                }
                std::string iol_site = get_ilogic_site(ctx->getBelName(io_bel).str(ctx));
                ci->attrs[id_BEL] = iol_site + "/ISERDESE2";
            }
        }



        else if (ci->type == id_IDDR) {
            fold_inverter(ci, "C");

            BelId io_bel;
            NetInfo *d = ci->getPort(id_D);
            if (d == nullptr || d->driver.cell == nullptr)
                log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
            CellInfo *drv = d->driver.cell;
            if (boost::contains(drv->type.str(ctx), "INBUF_EN") || boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                io_bel = ctx->getBelByNameStr(drv->attrs.at(id_BEL).as_string());
            else if (boost::contains(drv->type.str(ctx), "IDELAYE2") && d->driver.port == id_DATAOUT)
                io_bel = iodelay_to_io.at(drv->name);
            else
                log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                          ctx->nameOf(ci), drv->type.c_str(ctx));

            std::string iol_site = get_ilogic_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[id_BEL] = iol_site + "/IFF";
        } else if (ci->type == id_ISERDESE2) {
            fold_inverter(ci, "CLKB");
            fold_inverter(ci, "OCLKB");

            std::string iobdelay = str_or_default(ci->params, id_IOBDELAY, "NONE");
            BelId io_bel;

            if (iobdelay == "IFD") {
                NetInfo *d = ci->getPort(id_DDLY);
                if (d == nullptr || d->driver.cell == nullptr)
                    log_error("%s '%s' has disconnected DDLY input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                CellInfo *drv = d->driver.cell;
                if (boost::contains(drv->type.str(ctx), "IDELAYE2") && d->driver.port == id_DATAOUT)
                    io_bel = iodelay_to_io.at(drv->name);
                else
                    log_error("%s '%s' has DDLY input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                              ctx->nameOf(ci), drv->type.c_str(ctx));
            } else if (iobdelay == "NONE") {
                NetInfo *d = ci->getPort(id_D);
                if (d == nullptr || d->driver.cell == nullptr)
                    log_error("%s '%s' has disconnected D input\n", ci->type.c_str(ctx), ctx->nameOf(ci));
                CellInfo *drv = d->driver.cell;
                if (boost::contains(drv->type.str(ctx), "INBUF_EN") ||
                    boost::contains(drv->type.str(ctx), "INBUF_DCIEN"))
                    io_bel = ctx->getBelByNameStr(drv->attrs.at(id_BEL).as_string());
                else
                    log_error("%s '%s' has D input connected to illegal cell type %s\n", ci->type.c_str(ctx),
                              ctx->nameOf(ci), drv->type.c_str(ctx));
            } else {
                log_error("%s '%s' has unsupported IOBDELAY value '%s'\n", ci->type.c_str(ctx), ctx->nameOf(ci),
                          iobdelay.c_str());
            }

            std::string iol_site = get_ilogic_site(ctx->getBelName(io_bel).str(ctx));
            ci->attrs[id_BEL] = iol_site + "/ISERDESE2";
        }
    }

    flush_cells();
    generic_xform(iologic_rules, false);
    flush_cells();
}

void XC7Packer::pack_idelayctrl()
{
    CellInfo *idelayctrl = nullptr;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type == id_IDELAYCTRL) {
            if (idelayctrl != nullptr)
                log_error("Found more than one IDELAYCTRL cell!\n");
            idelayctrl = ci;
        }
    }
    if (idelayctrl == nullptr)
        return;
    std::set<std::string> ioctrl_sites;
    for (auto &cell : ctx->cells) {
        CellInfo *ci = cell.second.get();
        if (ci->type.in(id_IDELAYE2_IDELAYE2, id_ODELAYE2_ODELAYE2)) {
            if (!ci->attrs.count(id_BEL))
                continue;
            ioctrl_sites.insert(get_ioctrl_site(ci->attrs.at(id_X_IO_BEL).as_string()));
        }
    }
    if (ioctrl_sites.empty())
        log_error("Found IDELAYCTRL but no I/ODELAYs\n");
    NetInfo *rdy = idelayctrl->getPort(id_RDY);
    idelayctrl->disconnectPort(id_RDY);
    std::vector<NetInfo *> dup_rdys;
    int i = 0;
    for (auto site : ioctrl_sites) {
        auto dup_idc =
                create_cell(ctx, id_IDELAYCTRL, int_name(idelayctrl->name, "CTRL_DUP_" + std::to_string(i), false));
        dup_idc->connectPort(id_REFCLK, idelayctrl->getPort(id_REFCLK));
        dup_idc->connectPort(id_RST, idelayctrl->getPort(id_RST));
        if (rdy != nullptr) {
            NetInfo *dup_rdy =
                    (ioctrl_sites.size() == 1)
                            ? rdy
                            : create_internal_net(idelayctrl->name, "CTRL_DUP_" + std::to_string(i) + "_RDY", false);
            dup_idc->connectPort(id_RDY, dup_rdy);
            dup_rdys.push_back(dup_rdy);
        }
        dup_idc->attrs[id_BEL] = site + "/IDELAYCTRL";
        new_cells.push_back(std::move(dup_idc));
        ++i;
    }
    idelayctrl->disconnectPort(id_REFCLK);
    idelayctrl->disconnectPort(id_RST);

    if (rdy != nullptr) {
        // AND together all the RDY signals
        std::vector<NetInfo *> int_anded_rdy;
        int_anded_rdy.push_back(dup_rdys.front());
        for (size_t j = 1; j < dup_rdys.size(); j++) {
            NetInfo *anded_net =
                    (j == (dup_rdys.size() - 1))
                            ? rdy
                            : create_internal_net(idelayctrl->name, "ANDED_RDY_" + std::to_string(j), false);
            auto lut = create_lut(ctx, idelayctrl->name.str(ctx) + "/RDY_AND_LUT_" + std::to_string(j),
                                  {int_anded_rdy.at(j - 1), dup_rdys.at(j)}, anded_net, Property(8));
            int_anded_rdy.push_back(anded_net);
            new_cells.push_back(std::move(lut));
        }
    }

    packed_cells.insert(idelayctrl->name);
    flush_cells();

    ioctrl_rules[id_IDELAYCTRL].new_type = id_IDELAYCTRL_IDELAYCTRL;

    generic_xform(ioctrl_rules);
}

NEXTPNR_NAMESPACE_END
