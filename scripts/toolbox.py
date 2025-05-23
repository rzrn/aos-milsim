from itertools import islice
from random import randint
from time import time
from math import inf

from piqueserver.commands import command, player_only, handle_command, get_player
from piqueserver.config import config

from pyspades import contained as loaders
from pyspades.constants import *

def take(iterator, n, default = None):
    return next(islice(iterator, max(0, n - 1), None), default)

def ceildiv(n, d):
    q, r = divmod(n, d)
    return q + bool(r)

def randbyte():
    return randint(0, 255)

class StressPacket:
    def __init__(self, pid = None, length = None):
        self.id     = randint(0, 60) if pid is None else pid
        self.length = randint(0, 4096) if length is None else length

    def write(self, writer):
        writer.writeByte(self.id, True)

        for i in range(self.length):
            writer.writeByte(randbyte(), False)

@command()
@player_only
def stress(connection, pid = None, length = None):
    """
    Send random data with a given packet id
    /stress [packet id] [packet length]
    """

    try:
        if pid is not None:
            pid = int(pid)
    except ValueError:
        return "Packet id expected to be an integer"

    try:
        if length is not None:
            length = int(length)

            if length < 0:
                raise ValueError
    except ValueError:
        return "Packet length expected to be a positive integer"

    connection.send_contained(StressPacket(pid, length))

@command(admin_only = True)
def runas(connection, nickname, cmdname, *params):
    """
    Run command as other player
    /runas <nickname> <command> ...
    """

    player = get_player(connection.protocol, nickname)
    return handle_command(player, cmdname, params)

@command('listconnections', 'lscon')
def c_lscon(connection):
    """
    List players online
    /listconnections
    """

    protocol = connection.protocol
    return ", ".join(
        "#{}".format(player.player_id) if player.name is None else
        "{} (#{})".format(player.name, player.player_id)
        for player in protocol.connections.values()
    )

@command('ping')
def c_ping(connection, nickname = None):
    """
    Tell current ping of the given player (time for your actions to be received by the server)
    /ping [nickname]
    """

    player = connection if nickname is None else get_player(connection.protocol, nickname)

    if peer := getattr(player, 'peer', None):
        ENET_PEER_PACKET_LOSS_SCALE = (1 << 16) # TODO: should be moved to `enet`

        return "{nickname}: average = {average} ms, minimum = {minimum} ms, variance = {variance} ms, packet loss = {loss:.2f} %".format(
            nickname = player.name,
            average  = peer.roundTripTime,
            minimum  = peer.lowestRoundTripTime,
            variance = peer.roundTripTimeVariance,
            loss     = peer.packetLoss * 100 / ENET_PEER_PACKET_LOSS_SCALE
        )

mailbox   = config.section("mailbox")
mailfile  = mailbox.option("file", "mailbox.txt").get()
maildelay = mailbox.option("delay", 90).get()

@command('mail', 'admin')
@player_only
def mail(connection, *w):
    """
    Leave a message to the server administrator even if he is offline
    /mail <your message>
    """

    message = ' '.join(w).strip()

    if len(message) <= 0:
        return "Do not send empty messages (admins can see your IP)"

    ip, port = connection.address

    timestamp = time()

    dt = timestamp - getattr(connection, 'lastmail', -inf)

    if dt < maildelay:
        return "Do not write too often: wait {:.1f} seconds".format(maildelay - dt)

    with open(mailfile, 'a') as fout:
        fmtd = "{timestamp:.2f}: {nickname} ({ip}): {message}\n".format(
            timestamp = timestamp,
            nickname  = connection.name,
            ip        = ip,
            message   = message
        )

        fout.write(fmtd)
        connection.lastmail = timestamp

        return "Message sent"

def c_getattr(o, k, v):
    retval = getattr(o, k, v)
    setattr(o, k, retval)
    return retval

def c_globals(connection):
    ds = c_getattr(connection, 'globals', dict())

    protocol = connection.protocol

    ds.update(
        connection = connection,
        protocol   = protocol,
        idx        = protocol.players.get,
    )

    return ds

def format_exception(exc):
    return "{}: {}".format(type(exc).__name__, exc)

@command('eval', admin_only = True)
def c_eval(connection, *w):
    """
    Evaluate arbitrary Python code
    /eval <code>
    """

    expr = ' '.join(w)

    try:
        ds = c_globals(connection)

        retval = ds['_'] = eval(expr, ds)
        return str(retval)
    except Exception as exc:
        return format_exception(exc)

@command('exec', admin_only = True)
def c_exec(connection, *w):
    """
    Execute arbitrary Python code
    /exec <code>
    """

    stmt = ' '.join(w)

    try:
        exec(stmt, c_globals(connection))
    except Exception as exc:
        return format_exception(exc)

@command('delenv', admin_only = True)
def c_delenv(connection):
    """
    Clear /eval & /exec environment
    /delenv
    """

    c_globals(connection).clear()

from gc import collect
@command(admin_only = True)
def gc(connection):
    """
    Run the garbage collector
    /gc
    """

    return str(collect())

from piqueserver.commands import _alias_map, _commands

@command('listalias', 'alias', 'lsal')
def c_alias(connection, argval):
    """
    List all aliases to the given command
    /alias <command>
    """

    cmd = _alias_map.get(argval, argval)

    if cmd in _commands:
        cmds = ", ".join("/{}".format(k) for k, v in _alias_map.items() if v == cmd)
        return "{}: {}".format(cmd, cmds)
    else:
        return "Unknown command: {}".format(argval)

from piqueserver.commands import get_command_help

@command('help', 'info')
def c_help(connection, argval = None):
    """
    Gives description and usage info for a command
    /help <command name>
    """

    if argval is None:
        if msg := connection.protocol.help:
            connection.send_lines(msg, "help")

        return

    cmdname = _alias_map.get(argval, argval)

    if func := _commands.get(cmdname):
        desc, usage, _ = get_command_help(func)
        return "Description: {}\nUsage: {}".format(desc, usage)
    else:
        return "Unknown command: {}".format(argval)

@command('showrotation', 'shr')
def show_rotation(connection, argval = None):
    """
    Scroll through the current map rotation
    /shr [page number | query] or /shr * or /showrotation
    """

    page_size = 5

    maps = connection.protocol.get_map_rotation()
    total = ceildiv(len(maps), page_size)

    if argval == "*":
        return ", ".join(maps)

    npage = None

    if argval is None:
        npage = getattr(connection, 'show_rotation_page', 0)
    elif argval.isdigit():
        npage = max(1, min(total, int(argval))) - 1
    else:
        query = argval.lower()

        out = (i for i, map_name in enumerate(maps) if query in map_name.lower())

        if i := next(out, None):
            npage = i // page_size
        else:
            return "'{}' map not found".format(query)

    connection.show_rotation_page = (npage + 1) % total

    i1, i2 = npage * page_size, (npage + 1) * page_size
    return "{}/{}) {}".format(npage + 1, total, ", ".join(maps[i1 : i2]))

@command('advancemap', 'advance', 'adv', admin_only = True)
def advance(connection, argval = 1):
    """
    Force the next map to be immediately loaded instead of waiting for the time limit to end
    /advancemap [number of maps to skip] or /adv
    """

    protocol = connection.protocol

    if protocol.planned_map is None:
        protocol.planned_map = take(protocol.map_rotator, int(argval))

    protocol.advance_rotation('Map advance forced.')

@command('advancecancel', 'advca', 'adc', admin_only = True)
def advancecancel(connection):
    """
    Cancel map /advance
    /advancecancel or /adc
    """

    protocol = connection.protocol

    if defer := protocol.advance_deferred:
        if not defer.called:
            defer.cancel()

            protocol.broadcast_chat('Map advance cancelled.')

@command()
def mapname(connection):
    """
    Print the name of the current map
    /mapname
    """
    map_info = connection.protocol.map_info
    return "{} by {}".format(map_info.name, map_info.author)

@command('whatsnext', 'nextmap', 'wsn')
def c_whatsnext(connection):
    """
    Print name of the next map
    /whatsnext
    """

    protocol = connection.protocol

    if protocol.planned_map is None:
        protocol.planned_map = next(protocol.map_rotator)

    return "The next map is {}".format(protocol.planned_map.name)

def apply_script(protocol, connection, config):
    class ToolboxConnection(connection):
        def on_connect(self):
            self.chat_limiter._seconds = 1

            connection.on_connect(self)

        def existing_player_sent(self):
            return self.name is not None and self.team is not None

        def on_login(self, name):
            self.protocol.update_master()

            connection.on_login(self, name)

    from twisted.internet.defer import CancelledError

    class ToolboxProtocol(protocol):
        advance_deferred = None

        def advance_errback(self, failure):
            self.advance_deferred = None
            failure.trap(CancelledError)

        def advance_rotation(self, message = None):
            if defer := self.advance_deferred:
                defer.cancel()

            defer = protocol.advance_rotation(self, message)
            defer.addErrback(self.advance_errback)

            self.advance_deferred = defer
            return defer

        def get_player_count(self):
            return sum(connection.existing_player_sent() for connection in self.connections.values())

    return ToolboxProtocol, ToolboxConnection