<#
.SYNOPSIS
Run, record, inspect and develop the AutoDRIVE AVLite stack on Windows.
.DESCRIPTION
This is the shared entry point for the Windows tools. The command registry
selects a script in scripts/windows, and that script defines its own options.
Use a command with -Help to inspect its options without starting the operation.
.EXAMPLE
.\avlite.ps1 start
.EXAMPLE
.\avlite.ps1 laps -Laps 3 -Label controller-test
.EXAMPLE
.\avlite.ps1 response -Help
#>
[CmdletBinding(PositionalBinding = $false)]
param(
    [Parameter(Position = 0)]
    [ValidateSet('help', 'start', 'stop', 'restart', 'status', 'logs', 'laps',
        'record', 'response', 'speed', 'map', 'sketch-map', 'sketch-build',
        'sketch-test', 'test')]
    [string]$Command = 'help',
    [switch]$Help
)

dynamicparam {
    # PowerShell runs this block while binding arguments. Expose only the chosen
    # command's options, so a recording option cannot silently reach the launcher.
    . (Join-Path $PSScriptRoot 'scripts\windows\common.ps1')
    $commands = Get-AvliteCommands
    $parameters = [Management.Automation.RuntimeDefinedParameterDictionary]::new()
    if ($Command -ne 'help') {
        $entry = $commands[$Command]
        $scriptPath = Join-Path $PSScriptRoot ('scripts\windows\' + $entry.Script)
        $implementation = Get-Command -Name $scriptPath -CommandType ExternalScript
        # Read the implementation's declared options so defaults, types and
        # validation have one source. Never parse or evaluate command strings.
        foreach ($declared in $implementation.ScriptBlock.Ast.ParamBlock.Parameters) {
            $name = $declared.Name.VariablePath.UserPath
            if ($entry.Preset.ContainsKey($name)) { continue }
            $metadata = $implementation.Parameters[$name]
            $attributes = [Collections.ObjectModel.Collection[Attribute]]::new()
            $binding = [Management.Automation.ParameterAttribute]::new()
            foreach ($attribute in $metadata.Attributes) {
                if ($attribute -is [Management.Automation.ParameterAttribute]) {
                    if ($attribute.Mandatory -and -not $Help) { $binding.Mandatory = $true }
                } else {
                    $attributes.Add($attribute)
                }
            }
            # Child options are named; position zero belongs to Command.
            $attributes.Add($binding)
            $parameters.Add($name, [Management.Automation.RuntimeDefinedParameter]::new(
                $name, $metadata.ParameterType, $attributes))
        }
    }
    $parameters
}

end {
    $ErrorActionPreference = 'Stop'
    if ($Command -eq 'help') {
        Write-Output 'Usage: .\avlite.ps1 <command> [options]'
        Write-Output 'Use .\avlite.ps1 <command> -Help to see its options.'
        $commands.GetEnumerator() | ForEach-Object {
            [pscustomobject]@{ Command = $_.Key; Purpose = $_.Value.Description }
        } | Format-Table -AutoSize
        return
    }
    if ($Help) {
        Write-Output $entry.Description
        Write-Output "Usage: .\avlite.ps1 $Command [options]"
        foreach ($parameter in $parameters.Values) {
            $required = $implementation.Parameters[$parameter.Name].Attributes |
                Where-Object { $_ -is [Management.Automation.ParameterAttribute] -and $_.Mandatory }
            $suffix = if ($required) { ' (required)' } else { '' }
            Write-Output ('  -{0} <{1}>{2}' -f $parameter.Name, $parameter.ParameterType.Name, $suffix)
        }
        Write-Output 'Examples and defaults: docs/commands.md'
        return
    }
    $options = @{}
    # Forward only values the user supplied. Omitted options keep the defaults
    # in the child script; splatting also preserves numbers, switches and spaces.
    foreach ($name in $parameters.Keys) {
        if ($PSBoundParameters.ContainsKey($name)) { $options[$name] = $PSBoundParameters[$name] }
    }
    foreach ($name in $entry.Preset.Keys) { $options[$name] = $entry.Preset[$name] }
    & $scriptPath @options
}
