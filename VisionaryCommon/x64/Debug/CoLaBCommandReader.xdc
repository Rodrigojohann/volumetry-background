<?xml version="1.0"?><doc>
<members>
<member name="F:NETWORK_ERROR" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="24">
Network error (not sent with messages).
</member>
<member name="F:OK" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="27">
No error
</member>
<member name="F:METHOD_IN_ACCESS_DENIED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="30">
Wrong userlevel, access to method not allowed.
</member>
<member name="F:METHOD_IN_UNKNOWN_INDEX" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="33">
Trying to access a method with an unknown Sopas index.
</member>
<member name="F:VARIABLE_UNKNOWN_INDEX" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="36">
Trying to access a variable with an unknown Sopas index
</member>
<member name="F:LOCAL_CONDITION_FAILED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="39">
Local condition violated, e.g. giving a value that exceeds the minimum or maximum allowed value for this variable
</member>
<member name="F:INVALID_DATA" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="42">
Invalid data given for variable, this errorcode is deprecated (is not used anymore)
</member>
<member name="F:UNKNOWN_ERROR" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="45">
An error with unknown reason occurred, this errorcode is deprecated.
</member>
<member name="F:BUFFER_OVERFLOW" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="48">
The communication buffer was too small for the amount of data that should be serialised
</member>
<member name="F:BUFFER_UNDERFLOW" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="51">
More data was expected, the allocated buffer could not be filled.
</member>
<member name="F:ERROR_UNKNOWN_TYPE" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="54">
The variable that shall be serialised has an unknown type. This can only happen when there are variables in the firmware of the device that do not exist in the released description of the device. This should never happen.
</member>
<member name="F:VARIABLE_WRITE_ACCESS_DENIED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="57">
It is not allowed to write values to this variable. Probably the variable is defined as read-only
</member>
<member name="F:UNKNOWN_CMD_FOR_NAMESERVER" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="60">
When using names instead of indices, a command was issued that the nameserver does not understand
</member>
<member name="F:UNKNOWN_COLA_COMMAND" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="63">
The CoLa protocol specification does not define the given command, command is unknown
</member>
<member name="F:METHOD_IN_SERVER_BUSY" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="66">
It is not possible to issue more than one command at a time to an SRT device.
</member>
<member name="F:FLEX_OUT_OF_BOUNDS" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="69">
An array was accessed over its maximum length (the famous 0xE)
</member>
<member name="F:EVENT_REG_UNKNOWN_INDEX" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="72">
The event you wanted to register for does not exist, the index is unknown.
</member>
<member name="F:COLA_VALUE_UNDERFLOW" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="75">
The value does not fit into the value field, it is too large.
</member>
<member name="F:COLA_A_INVALID_CHARACTER" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="78">
Character is unknown, probably not alphanumeric (CoLaA only).
</member>
<member name="F:OSAI_NO_MESSAGE" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="81">
Only when using SRTOS in the firmware and distributed variables this error can occur. It is an indication that no operating system message could be created. This happens when trying to GET a variable
</member>
<member name="F:OSAI_NO_ANSWER_MESSAGE" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="84">
This is the same as OsaiNoMessage with the difference that it is thrown when trying to PUT a variable.
</member>
<member name="F:INTERNAL" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="87">
Internal error in the firmware, probably a pointer to a parameter was null.
</member>
<member name="F:HUB_ADDRESS_CORRUPTED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="90">
The Sopas Hubaddress is either too short or too long.
</member>
<member name="F:HUB_ADDRESS_DECODING" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="93">
The Sopas Hubaddress is invalid, it can not be decoded (Syntax).
</member>
<member name="F:HUB_ADDRESS_ADDRESS_EXCEEDED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="96">
Too many hubs in the address.
</member>
<member name="F:HUB_ADDRESS_BLANK_EXPECTED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="99">
When parsing a HubAddress an expected blank was not found. The HubAddress is not valid.
</member>
<member name="F:ASYNC_METHODS_ARE_SUPPRESSED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="102">
An asynchronous method call was made although the device was built with "AsyncMethodsSuppressed". This is an internal error that should never happen in a released device.
</member>
<member name="F:COMPLEX_ARRAYS_NOT_SUPPORTED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="105">
Device was built with "ComplexArraysSuppressed" because the compiler does not allow recursions. But now  a complex array was found. This is an internal error that should never happen in a released device.
</member>
<member name="F:SESSION_NO_RESOURCES" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="108">
CoLa2 session can not be created, no more sessions available.
</member>
<member name="F:SESSION_UNKNOWN_ID" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="111">
The CoLa2 session id is not valid, either it timed out or it never existed.
</member>
<member name="F:CANNOT_CONNECT" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="114">
Requested connection (probably to a Hub Device) could not be established.
</member>
<member name="F:INVALID_PORT" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="117">
The given PortId (for routing a CoLa2 telegram) does not exist.
</member>
<member name="F:SCAN_ALREADY_ACTIVE" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="120">
A UDP Scan is already running.
</member>
<member name="F:OUT_OF_TIMERS" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="123">
There are no more timer objects available (for SOPAS Scan).
</member>
<member name="F:WRITE_MODE_NOT_ENABLED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="126">
It is currently not allowed to write to the device, it is in RUN mode.
</member>
<member name="F:SET_PORT_FAILED" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="129">
Internal error with SOPAS Scan.
</member>
<member name="F:IO_LINK_FUNC_TEMP_NOT_AVAILABLE" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="132">
IoLink error: function temporarily not available.
</member>
<member name="F:UNKNOWN" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="135">
Unknown error, internally thrown if SOPAS Scan received an unknown command.
</member>
<member name="T:CoLaError.Enum" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaError.h" line="20">
Possible CoLa errors
</member>
<member name="M:CoLaBCommand.#ctor(CoLaCommandType.Enum,CoLaError.Enum,System.SByte!System.Runtime.CompilerServices.IsSignUnspecifiedByte!System.Runtime.CompilerServices.IsConst*)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="33">
<summary>Construct a new <see cref="T:CoLaBCommand"/> with the given command type, error, and name, but without any data.</summary>
</member>
<member name="M:CoLaBCommand.#ctor(std.vector&lt;System.Byte,std.allocator&lt;System.Byte&gt;&gt;)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="37">
<summary>Construct a new <see cref="T:CoLaBCommand"/> from the given data buffer.</summary>
</member>
<member name="M:CoLaBCommand.getBuffer" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="41">
<summary>Get the binary data buffer.</summary>
</member>
<member name="M:CoLaBCommand.getType" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="44">
<summary>Get the type of command.</summary>
</member>
<member name="M:CoLaBCommand.getName" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="47">
<summary>Get the name of command.</summary>
</member>
<member name="M:CoLaBCommand.getParameterOffset" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="50">
<summary>Get offset in bytes to where first parameter starts.</summary>
</member>
<member name="M:CoLaBCommand.getError" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="53">
<summary>Get error.</summary>
</member>
<member name="M:CoLaBCommand.networkErrorCommand" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommand.h" line="56">
<summary>Create a command for network errors.</summary>
</member>
<member name="T:CoLaBCommandReader" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="22">
<summary>
Class for reading data from a <see cref="T:CoLaBCommand"/>.
</summary>
</member>
<member name="M:CoLaBCommandReader.rewind" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="35">
<summary>
Rewind the position to the first parameter.
</summary>
</member>
<member name="M:CoLaBCommandReader.readSInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="40">
<summary>
Read a signed short int (8 bit, range [-128, 127]) and advances position by 1 byte.
</summary>
</member>
<member name="M:CoLaBCommandReader.readUSInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="45">
<summary>
Read a unsigned short int (8 bit, range [0, 255]) and advances position by 1 byte.
</summary>
</member>
<member name="M:CoLaBCommandReader.readInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="50">
<summary>
Read a signed int (16 bit, range [-32768, 32767]) and advances position by 2 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readUInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="55">
<summary>
Read a unsigned int (16 bit, range [0, 65535]) and advances position by 2 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readDInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="60">
<summary>
Read a signed double int (32 bit) and advances position by 4 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readUDInt" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="65">
<summary>
Read a unsigned int (32 bit) and advances position by 4 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readReal" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="70">
<summary>
Read a IEEE-754 single precision (32 bit) and advances position by 4 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readLReal" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="75">
<summary>
Read a IEEE-754 double precision (64 bit) and advances position by 8 bytes.
</summary>
</member>
<member name="M:CoLaBCommandReader.readBool" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="80">
<summary>
Read a boolean and advance the position by 1 byte.
</summary>
</member>
<member name="M:CoLaBCommandReader.readFlexString" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\CoLaBCommandReader.h" line="85">
<summary>
Read a flex string, and advance position according to string size.
</summary>
</member>
</members>
</doc>