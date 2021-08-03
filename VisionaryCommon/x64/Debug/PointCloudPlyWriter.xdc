<?xml version="1.0"?><doc>
<members>
<member name="T:PointCloudPlyWriter" decl="false" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\PointCloudPlyWriter.h" line="24">
<summary>Class for writing point clouds to PLY files.</summary>
</member>
<member name="M:PointCloudPlyWriter.WriteFormatPLY(System.SByte!System.Runtime.CompilerServices.IsSignUnspecifiedByte!System.Runtime.CompilerServices.IsConst*,std.vector&lt;PointXYZ,std.allocator&lt;PointXYZ&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,System.Boolean)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\PointCloudPlyWriter.h" line="29">
<summary>Save a point cloud to a file in Polygon File Format (PLY), see: https://en.wikipedia.org/wiki/PLY_%28file_format%29 </summary>
<param name="filename">The file to save the point cloud to</param>
<param name="points">The points to save</param>
<param name="useBinary">If the output file is binary or ascii</param>
<returns>Returns true if write was successful and false otherwise</returns>
</member>
<member name="M:PointCloudPlyWriter.WriteFormatPLY(System.SByte!System.Runtime.CompilerServices.IsSignUnspecifiedByte!System.Runtime.CompilerServices.IsConst*,std.vector&lt;PointXYZ,std.allocator&lt;PointXYZ&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,std.vector&lt;System.UInt32,std.allocator&lt;System.UInt32&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,System.Boolean)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\PointCloudPlyWriter.h" line="36">
<summary>Save a point cloud to a file in Polygon File Format (PLY) which has colors for each point, see: https://en.wikipedia.org/wiki/PLY_%28file_format%29 </summary>
<param name="filename">The file to save the point cloud to</param>
<param name="points">The points to save</param>
<param name="rgbaMap">RGBA colors for each point, must be same length as points</param>
<param name="useBinary">If the output file is binary or ascii</param>
<returns>Returns true if write was successful and false otherwise</returns>
</member>
<member name="M:PointCloudPlyWriter.WriteFormatPLY(System.SByte!System.Runtime.CompilerServices.IsSignUnspecifiedByte!System.Runtime.CompilerServices.IsConst*,std.vector&lt;PointXYZ,std.allocator&lt;PointXYZ&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,std.vector&lt;System.UInt16,std.allocator&lt;System.UInt16&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,System.Boolean)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\PointCloudPlyWriter.h" line="44">
<summary>Save a point cloud to a file in Polygon File Format (PLY) which has intensities for each point, see: https://en.wikipedia.org/wiki/PLY_%28file_format%29 </summary>
<param name="filename">The file to save the point cloud to</param>
<param name="points">The points to save</param>
<param name="intensityMap">Intensities for each point, must be same length as points</param>
<param name="useBinary">If the output file is binary or ascii</param>
<returns>Returns true if write was successful and false otherwise</returns>
</member>
<member name="M:PointCloudPlyWriter.WriteFormatPLY(System.SByte!System.Runtime.CompilerServices.IsSignUnspecifiedByte!System.Runtime.CompilerServices.IsConst*,std.vector&lt;PointXYZ,std.allocator&lt;PointXYZ&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,std.vector&lt;System.UInt32,std.allocator&lt;System.UInt32&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,std.vector&lt;System.UInt16,std.allocator&lt;System.UInt16&gt;&gt;!System.Runtime.CompilerServices.IsConst*!System.Runtime.CompilerServices.IsImplicitlyDereferenced,System.Boolean)" decl="true" source="C:\SICK\Integration\ProgrammingExamples\C++\TCP Blob Receiver\VisionaryCommon\PointCloudPlyWriter.h" line="52">
<summary>Save a point cloud to a file in Polygon File Format (PLY) which has intensities and colors for each point, see: https://en.wikipedia.org/wiki/PLY_%28file_format%29"&gt;Specification </summary>
<param name="filename">The file to save the point cloud to</param>
<param name="points">The points to save</param>
<param name="rgbaMap">RGBA colors for each point, must be same length as points</param>
<param name="intensityMap">Intensities for each point, must be same length as points</param>
<param name="useBinary">If the output file is binary or ascii</param>
<returns>Returns true if write was successful and false otherwise</returns>
</member>
</members>
</doc>